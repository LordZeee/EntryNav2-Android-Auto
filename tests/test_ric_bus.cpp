// Copyright (c) 2026 Jiraiya43
// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Tests platform/evo_ric_bus.cpp and the F-series arrows end to end against a fake BMW.RueKoIC (dbus-run-session).

// The fake owns BMW.RueKoIC on a private session bus, checks the path and interface of every call, records its member
// and model, and answers 0 (or a scripted answer). The arrows run with a G20 KOMBI address where nothing listens.
#include <dbus/dbus.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "evo_ric_bus.hpp"
#include "kombi_nav.hpp"
#include "kombi_test_util.hpp"

namespace {

using namespace evo::kombi;
using kombitest::Bytes;
using kombitest::Step;

pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;
std::vector<std::string> g_seen;  // "member type/id"
std::string g_refuse;             // a member answered with 7
volatile bool g_quit = false;
volatile int g_state = 0;  // 0 starting, 1 owns the name, 2 released it

std::string describe(DBusMessage* m) {
    std::string out = dbus_message_get_member(m);
    DBusMessageIter it;
    if (dbus_message_iter_init(m, &it) && dbus_message_iter_get_arg_type(&it) == DBUS_TYPE_UINT32) {
        dbus_uint32_t instance = 0, id = 0;
        dbus_int32_t type = -1;
        dbus_message_iter_get_basic(&it, &instance);
        if (dbus_message_iter_next(&it) && dbus_message_iter_get_arg_type(&it) == DBUS_TYPE_UINT32) {
            dbus_message_iter_get_basic(&it, &id);
            if (dbus_message_iter_next(&it) && dbus_message_iter_get_arg_type(&it) == DBUS_TYPE_INT32) {
                dbus_message_iter_get_basic(&it, &type);
            }
        }
        if (type >= 0) {  // ric_initialize (u) and ric_disposeResource (uu) carry no model
            char t[40];
            snprintf(t, sizeof(t), " %d/%u", static_cast<int>(type), static_cast<unsigned>(id));
            out += t;
        }
    }
    return out;
}

void* fakeRueKoIC(void*) {
    DBusError e;
    dbus_error_init(&e);
    DBusConnection* c = dbus_bus_get_private(DBUS_BUS_SESSION, &e);
    CHECK(c);
    dbus_connection_set_exit_on_disconnect(c, FALSE);
    while (g_state == 0) {
        usleep(10000);
    }
    CHECK(dbus_bus_request_name(c, "BMW.RueKoIC", 0, &e) == DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER);
    g_state = 3;  // owns it
    while (!g_quit && dbus_connection_read_write(c, 20)) {
        DBusMessage* m;
        while ((m = dbus_connection_pop_message(c)) != nullptr) {
            if (dbus_message_get_type(m) == DBUS_MESSAGE_TYPE_METHOD_CALL &&
                !strcmp(dbus_message_get_destination(m), "BMW.RueKoIC")) {
                CHECK(!strcmp(dbus_message_get_path(m), "/BMW/RueKoIC"));
                CHECK(!strcmp(dbus_message_get_interface(m),
                              "BMW.INFOTAINMENT.REMOTEINSTRUMENTCLUSTER.RICServiceGuidance"));
                pthread_mutex_lock(&g_mu);
                g_seen.push_back(describe(m));
                const bool refuse = !g_refuse.empty() && g_refuse == dbus_message_get_member(m);
                pthread_mutex_unlock(&g_mu);
                // The factory answers 48 to disposing a text that isn't there.
                dbus_int32_t answer = refuse ? 7 : !strcmp(dbus_message_get_member(m), "ric_disposeResource") ? 48 : 0;
                DBusMessage* r = dbus_message_new_method_return(m);
                dbus_message_append_args(r, DBUS_TYPE_INT32, &answer, DBUS_TYPE_INVALID);
                dbus_connection_send(c, r, nullptr);
                dbus_message_unref(r);
            }
            dbus_message_unref(m);
        }
        if (g_state == 4) {  // asked to release the name
            dbus_bus_release_name(c, "BMW.RueKoIC", &e);
            dbus_connection_flush(c);
            g_state = 2;
        }
    }
    dbus_connection_close(c);
    dbus_connection_unref(c);
    return nullptr;
}

size_t seenCount() {
    pthread_mutex_lock(&g_mu);
    const size_t n = g_seen.size();
    pthread_mutex_unlock(&g_mu);
    return n;
}

std::vector<std::string> seen() {
    pthread_mutex_lock(&g_mu);
    const std::vector<std::string> s = g_seen;
    pthread_mutex_unlock(&g_mu);
    return s;
}

bool waitFor(size_t n, double seconds = 8.0) {
    for (double t = 0; t < seconds; t += 0.02) {
        if (seenCount() >= n) {
            return true;
        }
        usleep(20000);
    }
    return false;
}

void clearSeen() {
    pthread_mutex_lock(&g_mu);
    g_seen.clear();
    pthread_mutex_unlock(&g_mu);
}

// A TCP port where nothing listens: the G20 link is refused at once.
uint16_t deadPort() {
    const int s = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    CHECK(bind(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);
    socklen_t len = sizeof(a);
    CHECK(getsockname(s, reinterpret_cast<sockaddr*>(&a), &len) == 0);
    close(s);
    return ntohs(a.sin_port);
}

std::vector<RicCall> aStart() {
    RouteGraph graph(0xC8, 255);
    std::vector<Request> r;
    CHECK(graph.routeStart(ClusterManeuver(kLeft, "Main Street", "300", kClusterMetres), r));
    RicGuidance g;
    std::vector<RicCall> calls;
    g.translate(r, calls);
    return calls;
}

void busSend() {
    CHECK(!ricBusAvailable());  // nobody owns BMW.RueKoIC yet
    std::string why;
    CHECK(ricBusSend(aStart(), &why) == kLinkDown && why.find("ric_setManeuverCtrlData") != std::string::npos);
    g_state = 1;
    for (int i = 0; i < 300 && g_state != 3; ++i) {
        usleep(10000);
    }
    CHECK(ricBusAvailable());

    clearSeen();
    const std::vector<RicCall> start = aStart();
    CHECK(ricBusSend(start, &why) == kSent && seenCount() == start.size());
    const std::vector<std::string> s = seen();
    CHECK(s.front() == "ric_setManeuverCtrlData 33/2" && s[2] == "ric_setResource 3/9000" &&
          s[20] == "ric_setManeuverData 15/200" && s[22] == "ric_initialize" &&
          s.back() == "ric_setManeuverCtrlData 33/2");

    RicGuidance g;
    std::vector<RicCall> teardown;
    g.teardown(teardown);
    clearSeen();
    CHECK(ricBusSend(teardown, &why) == kSent && seenCount() == 6);  // the 48 to the dispose is taken
    CHECK(seen()[1] == "ric_disposeResource");

    g_refuse = "ric_setManeuverListData";  // an answer other than 0: refused, and nothing after it
    clearSeen();
    CHECK(ricBusSend(start, &why) == kRefused);
    CHECK(why.find("answered 7 to ric_setManeuverListData") != std::string::npos);
    CHECK(seen().back() == "ric_setManeuverListData 32/1" && seenCount() == 22);
    g_refuse.clear();
    printf("PASS ric bus: no owner, start, teardown with 48, refusal\n");
}

struct Place {
    char dir[64];
    std::string off, marker, single, ric;
    Place() {
        snprintf(dir, sizeof(dir), "/tmp/kombi-ricXXXXXX");
        CHECK(mkdtemp(dir));
        off = std::string(dir) + "/kombi-off";
        marker = std::string(dir) + "/kombi-nav.shown";
        single = std::string(dir) + "/kombi-single-maneuver";
        ric = std::string(dir) + "/kombi-ric";
    }
    ~Place() {
        unlink(off.c_str());
        unlink(marker.c_str());
        unlink(single.c_str());
        unlink(ric.c_str());
        rmdir(dir);
    }
    ArrowsConfig config(uint16_t port) const {
        ArrowsConfig c;
        c.clusterAddress = "127.0.0.1";
        c.clusterPort = port;
        c.offFile = off;
        c.markerFile = marker;
        c.singleFile = single;
        c.ricFile = ric;
        c.guide.approachMiles = 0;
        return c;
    }
};

void tell(Arrows& arrows, const Step& step, int64_t metres, const char* text, int unit) {
    const Bytes status = kombitest::status(1), state = kombitest::state(step),
                position = kombitest::position(metres, text, unit);
    Arrows::onNavStatus(kNavStatus, &status[0], status.size(), &arrows);
    Arrows::onNavStatus(kNavState, &state[0], state.size(), &arrows);
    Arrows::onNavStatus(kNavPosition, &position[0], position.size(), &arrows);
}

bool exists(const std::string& path) {
    return access(path.c_str(), F_OK) == 0;
}

int g_bmw = 0;
int bmwProbe() {
    return g_bmw;
}

// No KOMBI on the Ethernet, BMW.RueKoIC on the bus: the arrows switch, show, update, clear.
void arrowsSwitchToRic() {
    for (int forced = 0; forced < 2; ++forced) {
        Place p;
        if (forced) {
            FILE* f = fopen(p.ric.c_str(), "w");
            CHECK(f);
            fclose(f);
        }
        clearSeen();
        {
            Arrows arrows(p.config(deadPort()));
            arrows.start(bmwProbe, &kSessionRicBus);
            tell(arrows, Step(7, "TEST ST"), 300, "300", kUnitMetres);
            CHECK(waitFor(24));
            CHECK(seen()[20] == "ric_setManeuverData 15/200");
            usleep(300000);
            CHECK(exists(p.marker));
            tell(arrows, Step(7, "TEST ST"), 200, "200", kUnitMetres);  // a new distance: one call
            CHECK(waitFor(25));
            CHECK(seen()[24] == "ric_setManeuverData 29/4000");
            tell(arrows, Step(8, "MAIN ST"), 500, "500", kUnitMetres);  // a new turn: teardown and start
            CHECK(waitFor(25 + 6 + 24));
            CHECK(seen()[25] == "ric_setManeuverCtrlData 33/2" && seen()[26] == "ric_disposeResource");
            g_bmw = 1;  // the BMW navigation guides: nothing more
            usleep(2600000);
            const size_t during = seenCount();
            tell(arrows, Step(8, "MAIN ST"), 400, "400", kUnitMetres);
            usleep(800000);
            CHECK(seenCount() == during);
            g_bmw = 0;
            tell(arrows, Step(8, "MAIN ST"), 300, "300", kUnitMetres);  // back: a whole start, no teardown first
            CHECK(waitFor(during + 24, 6.0));
            CHECK(seen()[during] == "ric_setManeuverCtrlData 33/2" && seen()[during + 2] == "ric_setResource 3/9000");
            arrows.onSessionEnded();  // the phone left: the teardown at once
            CHECK(waitFor(during + 24 + 6));
            usleep(400000);
            CHECK(!exists(p.marker));
            arrows.stop();
            CHECK(seenCount() == during + 24 + 6);  // nothing shown: nothing at exit
        }
        printf("PASS ric arrows: %s, show, distance, new turn, BMW yield, session end\n",
               forced ? "ric file" : "switched when no KOMBI answered");
    }

    // An arrow left by a killed run, no KOMBI: the teardown on BMW.RueKoIC at start
    Place p;
    FILE* f = fopen(p.marker.c_str(), "w");
    CHECK(f);
    fclose(f);
    clearSeen();
    {
        Arrows arrows(p.config(deadPort()));
        arrows.start(bmwProbe, &kSessionRicBus);
        CHECK(waitFor(6));
        usleep(300000);
        CHECK(!exists(p.marker) && seen()[0] == "ric_setManeuverCtrlData 33/2");
        arrows.stop();
    }
    printf("PASS ric arrows: teardown at start after a killed run\n");
}

// No KOMBI and no BMW.RueKoIC: G20 behaviour, nothing on the bus.
void noRueKoIC() {
    g_state = 4;
    for (int i = 0; i < 300 && g_state != 2; ++i) {
        usleep(10000);
    }
    CHECK(!ricBusAvailable());
    Place p;
    clearSeen();
    Arrows arrows(p.config(deadPort()));
    arrows.start(bmwProbe, &kSessionRicBus);
    tell(arrows, Step(7, "TEST ST"), 300, "300", kUnitMetres);
    usleep(1500000);
    arrows.stop();
    CHECK(seenCount() == 0);
    printf("PASS ric arrows: no BMW.RueKoIC, nothing on the bus\n");
}

}  // namespace

int main() {
    const char* address = getenv("DBUS_SESSION_BUS_ADDRESS");
    CHECK(address);
    setRicBusForTest(address);
    pthread_t fake;
    CHECK(pthread_create(&fake, nullptr, fakeRueKoIC, nullptr) == 0);
    busSend();
    arrowsSwitchToRic();
    noRueKoIC();
    g_quit = true;
    pthread_join(fake, nullptr);
    return 0;
}
