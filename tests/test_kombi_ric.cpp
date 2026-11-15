// Copyright (c) 2026 Jiraiya43
// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Tests platform/kombi_ric.cpp: the guide's G20 requests turned into the F-series calls, checked with libdbus itself.

// Every call is rebuilt in the wire format and read back by dbus_message_demarshal, which refuses anything malformed;
// its values are then read with libdbus's iterators. The sequences are compared with the factory's, call by call, as
// captured on an X1 F48: start, teardown, distance.
#include <dbus/dbus.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "kombi_display.hpp"
#include "kombi_graph.hpp"
#include "kombi_guide.hpp"
#include "kombi_ric.hpp"
#include "kombi_test_util.hpp"

namespace {

using namespace evo::kombi;
using kombitest::Step;

// One basic value read back from a call, depth first.
struct Token {
    char type;
    uint64_t number;
    double real;
    std::string text;
};

void flatten(DBusMessageIter* it, std::vector<Token>& out) {
    do {
        const int t = dbus_message_iter_get_arg_type(it);
        if (t == DBUS_TYPE_INVALID) {
            return;
        }
        if (t == DBUS_TYPE_STRUCT || t == DBUS_TYPE_ARRAY || t == DBUS_TYPE_VARIANT) {
            DBusMessageIter sub;
            dbus_message_iter_recurse(it, &sub);
            flatten(&sub, out);
            continue;
        }
        Token k;
        k.type = static_cast<char>(t);
        k.number = 0;
        k.real = 0;
        if (t == DBUS_TYPE_STRING) {
            const char* s = nullptr;
            dbus_message_iter_get_basic(it, &s);
            k.text = s;
        } else if (t == DBUS_TYPE_DOUBLE) {
            dbus_message_iter_get_basic(it, &k.real);
        } else {
            DBusBasicValue v;
            v.u64 = 0;
            dbus_message_iter_get_basic(it, &v);
            k.number = t == DBUS_TYPE_BYTE     ? v.byt
                       : t == DBUS_TYPE_UINT16 ? v.u16
                       : t == DBUS_TYPE_UINT64 ? v.u64
                       : t == DBUS_TYPE_INT32  ? static_cast<uint32_t>(v.i32)
                                               : v.u32;
        }
        out.push_back(k);
    } while (dbus_message_iter_next(it));
}

// A call as libdbus reads it.
struct Seen {
    std::string member, signature, path, interface, destination;
    uint32_t id, type;
    std::vector<Token> tokens;
    Seen() : id(0), type(0) {}
    std::string text() const {  // the first string
        for (size_t i = 0; i < tokens.size(); ++i) {
            if (tokens[i].type == 's') {
                return tokens[i].text;
            }
        }
        return "<none>";
    }
    uint64_t numberAfterText() const {
        for (size_t i = 0; i + 1 < tokens.size(); ++i) {
            if (tokens[i].type == 's') {
                return tokens[i + 1].number;
            }
        }
        return 999;
    }
};

Seen read(const RicCall& c) {
    const Bytes wire = ricWireMessage(c, 7);
    DBusError e;
    dbus_error_init(&e);
    DBusMessage* m = dbus_message_demarshal(reinterpret_cast<const char*>(&wire[0]), static_cast<int>(wire.size()), &e);
    if (!m) {
        fprintf(stderr, "demarshal %s: %s\n", c.member.c_str(), e.message ? e.message : "?");
    }
    CHECK(m);
    Seen s;
    s.member = dbus_message_get_member(m);
    s.signature = dbus_message_get_signature(m);
    s.path = dbus_message_get_path(m);
    s.interface = dbus_message_get_interface(m);
    s.destination = dbus_message_get_destination(m);
    CHECK(dbus_message_get_type(m) == DBUS_MESSAGE_TYPE_METHOD_CALL && !dbus_message_get_sender(m));
    CHECK(s.signature == c.signature);
    DBusMessageIter it;
    if (dbus_message_iter_init(m, &it)) {
        flatten(&it, s.tokens);
    }
    if (s.signature == "uui(yv)") {
        s.id = static_cast<uint32_t>(s.tokens[1].number);
        s.type = static_cast<uint32_t>(s.tokens[2].number);
    } else if (s.signature == "uu") {
        s.id = static_cast<uint32_t>(s.tokens[1].number);
    }
    dbus_message_unref(m);
    dbus_error_free(&e);
    return s;
}

std::vector<Seen> readAll(const std::vector<RicCall>& calls) {
    std::vector<Seen> out;
    for (size_t i = 0; i < calls.size(); ++i) {
        out.push_back(read(calls[i]));
        CHECK(out.back().path == "/BMW/RueKoIC" && out.back().destination == "BMW.RueKoIC" &&
              out.back().interface == "BMW.INFOTAINMENT.REMOTEINSTRUMENTCLUSTER.RICServiceGuidance");
    }
    return out;
}

// member/type/id, the way the factory's calls were listed from the capture.
std::string shape(const std::vector<Seen>& s) {
    std::string out;
    for (size_t i = 0; i < s.size(); ++i) {
        char one[80];
        if (s[i].member == "ric_initialize") {
            snprintf(one, sizeof(one), "init ");
        } else if (s[i].member == "ric_disposeResource") {
            snprintf(one, sizeof(one), "dispose:%u ", s[i].id);
        } else {
            snprintf(one, sizeof(one), "%u/%u ", s[i].type, s[i].id);
        }
        out += one;
    }
    return out;
}

// The factory's start on the F48 (state, car info, text, road, distance, thresholds, graph, order, six
// sub-distances with their events, description, list, init, state) and its teardown.
const char* const kStart =
    "33/2 17/3 3/9000 40/800 29/4000 34/700 31/1300 7/1200 29/4001 36/15000 29/4002 36/15001 29/4003 36/15002 "
    "29/4004 36/15003 29/4005 36/15004 29/4006 36/15005 15/200 32/1 init 33/2 ";
const char* const kTeardown = "33/2 dispose:9000 33/2 32/1 17/3 init ";

int stateOf(const Seen& s) {  // guidance state model: ((2,33), active, -1, state, ...)
    return static_cast<int>(s.tokens[8].number);
}
uint32_t activeOf(const Seen& s) {
    return static_cast<uint32_t>(s.tokens[6].number);
}
int directionOf(const Seen& description) {  // (((200,15),0,0,-1,0,-1), direction, ...)
    return static_cast<int>(description.tokens[11].number);
}
size_t listCount(const Seen& list) {  // ((1,32), 0, n, n, ids, ...)
    return static_cast<size_t>(list.tokens[7].number);
}

const Seen& find(const std::vector<Seen>& s, uint32_t type, uint32_t id) {
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i].type == type && s[i].id == id) {
            return s[i];
        }
    }
    CHECK(false);
    return s[0];
}

std::vector<Request> routeStart(int direction, const std::string& road, const std::string& distance, int unit) {
    RouteGraph graph(0xC8, 255);
    std::vector<Request> r;
    CHECK(graph.routeStart(ClusterManeuver(direction, road, distance, unit), r));
    return r;
}

void checkStart(const std::vector<Seen>& s, int direction, const std::string& road, const std::string& distance,
                int unit) {
    CHECK(shape(s) == kStart);
    CHECK(stateOf(s.front()) == 1 && activeOf(s.front()) == 0xFFFFFFFFu);
    CHECK(stateOf(s.back()) == 2 && activeOf(s.back()) == 200);
    CHECK(find(s, 3, 9000).text() == road);
    const Seen& d = find(s, 29, 4000);
    CHECK(d.text() == distance && d.numberAfterText() == static_cast<uint64_t>(unit));
    CHECK(d.tokens[6].number == 255);  // the red bar hidden, as the guide asks
    CHECK(directionOf(find(s, 15, 200)) == direction);
    CHECK(listCount(find(s, 32, 1)) == 1);
}

// Every direction the guide can ask for (kombi_display.hpp), metric and imperial.
void everyDirection() {
    const int directions[] = {kStraightOn,
                              kHalfLeft,
                              kHalfRight,
                              kTightLeft,
                              kTightRight,
                              kLeft,
                              kRight,
                              kUTurnLeft,
                              kUTurnRight,
                              kDestination,
                              kRoundaboutClockwise,
                              kRoundaboutCounterClockwise,
                              kHighwayInLeft,
                              kHighwayInRight,
                              kHighwayOutLeft,
                              kHighwayOutRight,
                              kLaneChangeLeft,
                              kLaneChangeRight,
                              kForkLeft,
                              kForkRight};
    for (size_t i = 0; i < sizeof(directions) / sizeof(directions[0]); ++i) {
        for (int imperial = 0; imperial < 2; ++imperial) {
            const std::string distance = imperial ? "0.5" : "300";
            const int unit = imperial ? kClusterMiles : kClusterMetres;
            RicGuidance g;
            std::vector<RicCall> calls;
            g.translate(routeStart(directions[i], "Main Street", distance, unit), calls);
            checkStart(readAll(calls), directions[i], "Main Street", distance, unit);
            CHECK(g.showing() && g.shown().direction == directions[i]);
        }
    }
    printf("PASS ric: start for %u directions, metric and imperial\n",
           static_cast<unsigned>(sizeof(directions) / sizeof(directions[0])));
}

void distanceThenNewTurnThenCancel() {
    RouteGraph graph(0xC8, 255);
    RicGuidance g;
    std::vector<RicCall> calls;
    g.translate(routeStart(kLeft, "Main Street", "300", kClusterMetres), calls);
    CHECK(calls.size() == 24);

    std::vector<Request> r;
    CHECK(graph.distanceUpdate("150", kClusterMetres, r));
    g.translate(r, calls);
    std::vector<Seen> s = readAll(calls);
    CHECK(shape(s) == "29/4000 " && s[0].text() == "150" && s[0].numberAfterText() == kClusterMetres);
    g.translate(r, calls);  // the same distance again: nothing
    CHECK(calls.empty());

    CHECK(graph.positionUpdate(1234.0, r));  // a route position: nothing on the F-series
    g.translate(r, calls);
    CHECK(calls.empty());

    // The same turn and road again, a new distance (an update in place): the distance alone
    CHECK(graph.maneuverUpdate(ClusterManeuver(kLeft, "Main Street", "100", kClusterMetres), true, r));
    g.translate(r, calls);
    s = readAll(calls);
    CHECK(shape(s) == "29/4000 " && s[0].text() == "100");

    // A new turn: the factory's teardown, then a whole start
    CHECK(graph.maneuverUpdate(ClusterManeuver(kRight, "Oak Avenue", "1.2", kClusterKilometres), true, r));
    g.translate(r, calls);
    s = readAll(calls);
    CHECK(shape(s) == std::string(kTeardown) + kStart);
    CHECK(stateOf(s[0]) == 5 && activeOf(s[0]) == 200 && stateOf(s[2]) == 0 && listCount(s[3]) == 0);
    CHECK(s[4].text() == "");  // car info zeroed
    const std::vector<Seen> start(s.begin() + 6, s.end());
    checkStart(start, kRight, "Oak Avenue", "1.2", kClusterKilometres);
    CHECK(calls[1].unknownOk);  // disposing a text that isn't there is fine

    // The guide's cancel: the teardown, once
    graph.cancel(r);
    g.translate(r, calls);
    CHECK(shape(readAll(calls)) == kTeardown && !g.showing());
    g.translate(r, calls);
    CHECK(calls.empty());
    g.teardown(calls);  // asked for anyway (an arrow left by a killed run)
    CHECK(shape(readAll(calls)) == kTeardown);

    // A distance before any turn: nothing
    graph.distanceUpdate("90", kClusterMetres, r);
    g.translate(r, calls);
    CHECK(calls.empty() && !g.showing());

    // The BMW navigation took over: no teardown of ours before the next start
    g.translate(routeStart(kLeft, "Main Street", "300", kClusterMetres), calls);
    g.forget();
    g.translate(routeStart(kRight, "Main Street", "300", kClusterMetres), calls);
    CHECK(shape(readAll(calls)) == kStart);

    // A failed batch: the next change starts with a teardown
    g.assumeShown();
    g.translate(routeStart(kRight, "Other", "300", kClusterMetres), calls);
    CHECK(shape(readAll(calls)) == std::string(kTeardown) + kStart);
    printf("PASS ric: distance, new turn, cancel, yield, failed batch\n");
}

void textsCut() {
    // 30 Cyrillic letters: 20 kept, never half a character
    std::string road;
    for (int i = 0; i < 30; ++i) {
        road += "\xd0\x96";  // Ж
    }
    RicGuidance g;
    std::vector<RicCall> calls;
    g.translate(routeStart(kLeft, road, "12345678", kClusterMetres), calls);
    const std::vector<Seen> s = readAll(calls);
    CHECK(find(s, 3, 9000).text() == road.substr(0, 40));
    CHECK(find(s, 29, 4000).text() == "123456");
    CHECK(ricCut("ab\xf0\x9f\x98\x80"
                 "cd",
                 3) == "ab\xf0\x9f\x98\x80");
    CHECK(ricCut("abc", 0).empty() && ricCut("", 5).empty());
    CHECK(ricCut("a\xd0", 5) == "a");  // a cut character at the end is dropped
    printf("PASS ric: texts cut to %u and %u characters\n", static_cast<unsigned>(kRicRoadChars),
           static_cast<unsigned>(kRicDistanceChars));
}

// The guide's own output, translated: the F-series side of a whole drive.
class RicRecorder : public GuideOutput {
public:
    SendOutcome send(const std::vector<Request>& requests, std::string*) override {
        std::vector<RicCall> calls;
        ric.translate(requests, calls);
        add(calls);
        return kSent;
    }
    bool cancel(std::string*) override {
        std::vector<RicCall> calls;
        ric.teardown(calls);
        add(calls);
        return true;
    }
    void add(const std::vector<RicCall>& calls) {
        const std::vector<Seen> s = readAll(calls);
        seen.insert(seen.end(), s.begin(), s.end());
    }
    RicGuidance ric;
    std::vector<Seen> seen;
};

// Left, right, a roundabout, a slight left, the destination; every turn shown in the factory's order, never a start
// over a shown maneuver, cleared at the end.
void wholeDrive() {
    for (int mode = 0; mode < 2; ++mode) {
        GuideSettings settings;  // the service's defaults
        settings.inPlace = mode ? GuideSettings::kInPlace : GuideSettings::kFullGraphs;
        settings.approachMiles = 0;
        RouteGraph graph(0xC8, 255);
        RicRecorder out;
        Guide guide(graph, out, settings);
        const std::vector<kombitest::Moment> drive = kombitest::drive(kombitest::demoRoute());
        for (size_t i = 0; i < drive.size(); ++i) {
            Maneuver m;
            const bool shown = drive[i].inbox.guiding() && drive[i].inbox.nextManeuver(m);
            guide.step(shown ? &m : nullptr, drive[i].t);
        }
        guide.step(nullptr, drive.back().t + 100);
        CHECK(!guide.showing() && !out.ric.showing());

        bool up = false;
        std::vector<int> turns;
        std::vector<std::string> roads;
        size_t distances = 0;
        for (size_t i = 0; i < out.seen.size(); ++i) {
            const Seen& c = out.seen[i];
            if (c.type == 33 && stateOf(c) == 1) {
                CHECK(!up);  // a start only when nothing is shown
                CHECK(shape(std::vector<Seen>(out.seen.begin() + i, out.seen.begin() + i + 24)) == kStart);
                turns.push_back(directionOf(out.seen[i + 20]));
                roads.push_back(out.seen[i + 2].text());
                up = true;
            } else if (c.type == 33 && stateOf(c) == 5) {
                CHECK(up);
                CHECK(shape(std::vector<Seen>(out.seen.begin() + i, out.seen.begin() + i + 6)) == kTeardown);
                up = false;
            } else if (c.type == 29 && c.id == 4000 && !(i >= 4 && out.seen[i - 4].type == 33)) {
                CHECK(up);  // a distance only on a shown maneuver
                ++distances;
            }
        }
        CHECK(!up);
        const int expected[] = {kLeft, kRight, kRoundaboutCounterClockwise, kHalfLeft, kDestination};
        for (size_t i = 0; i < turns.size(); ++i) {
            printf("  turn %u: direction %d, road '%s'\n", static_cast<unsigned>(i), turns[i], roads[i].c_str());
        }
        CHECK(turns == std::vector<int>(expected, expected + 5));
        CHECK(roads[0] == "TEST ST" && roads[1] == "MAIN ST" && roads[4] == "DESTINATION");
        CHECK(roads[2].find("CIRCLE RD") != std::string::npos);
        CHECK(distances > 20);
        printf("PASS ric: whole drive (%s), %u turns, %u distance updates, %u calls\n",
               mode ? "updates in place" : "full graphs", static_cast<unsigned>(turns.size()),
               static_cast<unsigned>(distances), static_cast<unsigned>(out.seen.size()));
    }
}

}  // namespace

int main() {
    everyDirection();
    distanceThenNewTurnThenCancel();
    textsCut();
    wholeDrive();
    return 0;
}
