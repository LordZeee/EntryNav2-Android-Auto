// Copyright (c) 2026 Jiraiya43
// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Tests platform/kombi_ric.cpp: the guide's G20 requests turned into the F-series calls, checked with libdbus itself.

// Every call is rebuilt in the wire format and read back by dbus_message_demarshal, which refuses anything malformed;
// its values are then read with libdbus's iterators. The sequences are compared with the factory's, call by call, as
// captured on an X1 F48: start, teardown, distance.
#include <dbus/dbus.h>
#include <math.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "kombi_display.hpp"
#include "kombi_graph.hpp"
#include "kombi_guide.hpp"
#include "kombi_ric.hpp"
#include "kombi_ric_template.hpp"
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
    CHECK(directionOf(find(s, 15, 200)) == ricHudDirection(direction));
    CHECK(find(s, 15, 200).tokens[14].number == 1200);  // the junction element
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

// The exit's end (point 6) of a graph body, read back with libdbus: (lat, lon).
void exitPoint(const Bytes& body, double& lat, double& lon) {
    const Seen s = read(RicCall("ric_setGuidingData", "uui(yv)", body));
    size_t reals = 0;
    for (size_t i = 0; i < s.tokens.size(); ++i) {
        if (s.tokens[i].type == 'd') {
            ++reals;
        }
    }
    CHECK(reals == 5 + 12);  // five element widths, six points of two coordinates
    size_t seen = 0;
    for (size_t i = 0; i < s.tokens.size(); ++i) {
        if (s.tokens[i].type == 'd' && ++seen == reals - 1) {
            lat = s.tokens[i].real;
            lon = s.tokens[i + 1].real;
        }
    }
}

// The junction: the template's own points give the captured graph byte for byte; each direction gets its exit road at
// its angle (checked on the car: -90 left, 0 straight, 45 half right, -135 left); the start sends that graph.
void junctions() {
    rictmpl::Call graph = rictmpl::after_distance[0];
    for (size_t i = 0; i < sizeof(rictmpl::after_distance) / sizeof(rictmpl::after_distance[0]); ++i) {
        if (rictmpl::after_distance[i].body == rictmpl::graph) {
            graph = rictmpl::after_distance[i];
        }
    }
    const double lat[6] = {
        50.0, 50.000225, 50.00023681285804, 50.000330002368415, 50.00023681285804, 50.00023203676723};
    const double lon[6] = {10.0, 10.0, 9.999748189950346, 9.9997, 9.9997, 10.0003};
    CHECK(ricGraphBody(lat, lon) == Bytes(graph.body, graph.body + graph.len));

    struct Expect {
        int direction;
        double angle;
    } const expect[] = {{kStraightOn, 0},         {kHalfLeft, -45},  {kHalfRight, 45},       {kTightLeft, -135},
                        {kTightRight, 135},       {kLeft, -90},      {kRight, 90},           {kUTurnLeft, 180},
                        {kUTurnRight, 180},       {kDestination, 0}, {kHighwayOutLeft, -30}, {kHighwayOutRight, 30},
                        {kForkLeft, -30},         {kForkRight, 30},  {kLaneChangeLeft, -20}, {kLaneChangeRight, 20},
                        {kRoundaboutClockwise, 0}};
    for (size_t i = 0; i < sizeof(expect) / sizeof(expect[0]); ++i) {
        CHECK(ricExitAngle(expect[i].direction) == expect[i].angle);
        double la = 0, lo = 0;
        exitPoint(ricTurnGraph(expect[i].angle), la, lo);
        const double north = (la - 50.000225) * 111320.0, east = (lo - 10.0) * 71555.0;
        const double bearing = atan2(east, north) * 180.0 / 3.14159265358979;
        CHECK(fabs(sqrt(north * north + east * east) - 25.0) < 0.01);
        CHECK(fabs(bearing - expect[i].angle) < 0.01 || fabs(fabs(bearing) - 180.0) < 0.01);

        RicGuidance g;
        std::vector<RicCall> calls;
        g.translate(routeStart(expect[i].direction, "Main Street", "300", kClusterMetres), calls);
        int graphs = 0;
        for (size_t k = 0; k < calls.size(); ++k) {
            if (calls[k].member == "ric_setGuidingData" && read(calls[k]).type == 31) {
                CHECK(calls[k].body == ricTurnGraph(expect[i].angle));
                ++graphs;
            }
        }
        CHECK(graphs == 1);
    }
    // Left turns put the side roads on the right: none of them west of the junction.
    const Seen left = read(RicCall("ric_setGuidingData", "uui(yv)", ricTurnGraph(-90)));
    for (size_t i = 0; i < left.tokens.size(); ++i) {
        if (left.tokens[i].type == 'd' && left.tokens[i].real > 9.0 && left.tokens[i].real < 11.0) {
            CHECK(left.tokens[i].real >= 10.0 - 1e-9 || fabs(left.tokens[i].real - (10.0 - 25.0 / 71555.0)) < 1e-9);
        }
    }
    printf("PASS ric: junction graph byte for byte as captured, exit road per direction (%u directions)\n",
           static_cast<unsigned>(sizeof(expect) / sizeof(expect[0])));
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

// The graph a start sent, or empty.
Bytes graphOf(const std::vector<RicCall>& calls) {
    for (size_t k = 0; k < calls.size(); ++k) {
        if (calls[k].member == "ric_setGuidingData" && read(calls[k]).type == 31) {
            return calls[k].body;
        }
    }
    return Bytes();
}

// Roundabouts: the exit road at the phone's angle, or a quarter turn per exit without one; "Sortie" as "Exit".
void roundabouts() {
    CHECK(ricRoundaboutTurn(kRoundaboutCounterClockwise, 90) == 90);    // first exit, a right turn
    CHECK(ricRoundaboutTurn(kRoundaboutCounterClockwise, 180) == 0);    // straight on
    CHECK(ricRoundaboutTurn(kRoundaboutCounterClockwise, 270) == -90);  // a left turn
    CHECK(ricRoundaboutTurn(kRoundaboutCounterClockwise, 350) == -160);
    CHECK(ricRoundaboutTurn(kRoundaboutClockwise, 90) == -90);  // left-hand traffic, mirrored
    CHECK(ricRoundaboutTurn(kRoundaboutCounterClockwise, 1) == 160);
    CHECK(ricExitNumber("Exit 3 - RUE") == 3 && ricExitNumber("Exit 12") == 12 && ricExitNumber("RUE") == 0);

    RicGuidance g;
    std::vector<RicCall> calls;
    g.setRoundaboutHint(true, 270);
    g.translate(routeStart(kRoundaboutCounterClockwise, "Sortie 3 - RUE A", "300", kClusterMetres), calls);
    CHECK(find(readAll(calls), 3, 9000).text() == "Exit 3 - RUE A");
    CHECK(graphOf(calls) == ricTurnGraph(-90) && g.shown().angle == -90);

    // No angle from the phone: from the exit number on the road line
    RicGuidance h;
    h.translate(routeStart(kRoundaboutCounterClockwise, "Sortie 1", "300", kClusterMetres), calls);
    CHECK(graphOf(calls) == ricTurnGraph(90));
    RicGuidance n;
    n.translate(routeStart(kRoundaboutCounterClockwise, "RUE", "300", kClusterMetres), calls);
    CHECK(graphOf(calls) == ricTurnGraph(0));

    // A new angle for the same roundabout is a new turn
    RouteGraph graph(0xC8, 255);
    std::vector<Request> r;
    g.setRoundaboutHint(true, 180);
    CHECK(graph.distanceUpdate("250", kClusterMetres, r));
    g.translate(r, calls);
    CHECK(shape(readAll(calls)) == std::string(kTeardown) + kStart && graphOf(calls) == ricTurnGraph(0));
    // The hint is for roundabouts only
    RicGuidance o;
    o.setRoundaboutHint(true, 270);
    o.translate(routeStart(kRight, "Sortie 3", "300", kClusterMetres), calls);
    CHECK(graphOf(calls) == ricTurnGraph(90));
    printf("PASS ric: roundabout exit from the phone's angle or the exit number, Exit text\n");
}

// The phone flipping between two maneuvers: a new turn within kRicTurnHoldS of the last start waits, and is dropped
// when the phone flips back.
void flipFlop() {
    RouteGraph graph(0xC8, 255);
    std::vector<Request> r;
    RicGuidance g;
    std::vector<RicCall> calls;
    g.translate(routeStart(kLeft, "A", "300", kClusterMetres), calls, 10.0);
    CHECK(shape(readAll(calls)) == kStart);

    // Flip to a right turn and back within the hold: nothing on the HUD
    CHECK(graph.maneuverUpdate(ClusterManeuver(kRight, "B", "300", kClusterMetres), true, r));
    g.translate(r, calls, 10.5);
    CHECK(calls.empty() && g.pending());
    CHECK(graph.distanceUpdate("280", kClusterMetres, r));  // a distance for the held turn: still held
    g.translate(r, calls, 10.8);
    CHECK(calls.empty() && g.pending());
    g.flush(11.0, calls);
    CHECK(calls.empty() && g.pending());
    CHECK(graph.maneuverUpdate(ClusterManeuver(kLeft, "A", "290", kClusterMetres), true, r));
    g.translate(r, calls, 11.2);
    CHECK(shape(readAll(calls)) == "29/4000 " && !g.pending() && g.shown().direction == kLeft);
    g.flush(13.0, calls);
    CHECK(calls.empty());

    // A new turn the phone keeps: shown by flush once the hold is over, with its latest distance
    CHECK(graph.maneuverUpdate(ClusterManeuver(kRight, "B", "300", kClusterMetres), true, r));
    g.translate(r, calls, 11.3);
    CHECK(calls.empty() && g.pending());
    CHECK(graph.distanceUpdate("270", kClusterMetres, r));
    g.translate(r, calls, 11.4);
    g.flush(11.49, calls);
    CHECK(calls.empty());
    g.flush(11.6, calls);
    std::vector<Seen> s = readAll(calls);
    CHECK(shape(s) == std::string(kTeardown) + kStart && !g.pending());
    CHECK(g.shown().direction == kRight && find(s, 29, 4000).text() == "270");

    // Long after the last start: at once
    CHECK(graph.maneuverUpdate(ClusterManeuver(kHalfLeft, "C", "300", kClusterMetres), true, r));
    g.translate(r, calls, 20.0);
    CHECK(shape(readAll(calls)) == std::string(kTeardown) + kStart);

    // A cancel drops a held turn
    CHECK(graph.maneuverUpdate(ClusterManeuver(kRight, "D", "300", kClusterMetres), true, r));
    g.translate(r, calls, 20.5);
    CHECK(g.pending());
    graph.cancel(r);
    g.translate(r, calls, 20.6);
    CHECK(shape(readAll(calls)) == kTeardown && !g.pending() && !g.showing());
    g.flush(30.0, calls);
    CHECK(calls.empty());
    printf("PASS ric: flip-flops held %.1f s, dropped on a flip back, kept turns shown, cancel drops\n", kRicTurnHoldS);
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

// The red bar from the phone's metres: L fixed at the turn's start (at most 700 m), fill 100 (L - D) / L.
void bar() {
    CHECK(ricBar(700, 900) == 0 && ricBar(700, 700) == 0 && ricBar(700, 350) == 50 && ricBar(700, 0) == 100);
    CHECK(ricBar(300, 75) == 75 && ricBar(0, 10) == 0 && ricBar(200, -5) == 100);

    RicGuidance g;
    std::vector<RicCall> calls;
    g.setMetresToTurn(true, 1200);
    g.translate(routeStart(kLeft, "A", "1.2", kClusterKilometres), calls, 1.0);
    CHECK(find(readAll(calls), 29, 4000).tokens[6].number == 0);  // 1.2 km: the bar not started
    g.setMetresToTurn(true, 525);
    g.refreshBar(1.2, calls);
    CHECK(calls.empty());  // at most every half second
    g.refreshBar(1.6, calls);
    std::vector<Seen> s = readAll(calls);
    CHECK(shape(s) == "29/4000 " && s[0].tokens[6].number == 25 && s[0].text() == "1.2");
    g.refreshBar(2.5, calls);
    CHECK(calls.empty());  // no change
    // A new turn 300 m away: L = 300
    RouteGraph graph(0xC8, 255);
    std::vector<Request> r;
    g.setMetresToTurn(true, 300);
    CHECK(graph.maneuverUpdate(ClusterManeuver(kRight, "B", "300", kClusterMetres), true, r));
    g.translate(r, calls, 10.0);
    CHECK(find(readAll(calls), 29, 4000).tokens[6].number == 0);
    g.setMetresToTurn(true, 75);
    CHECK(graph.distanceUpdate("75", kClusterMetres, r));
    g.translate(r, calls, 11.0);
    s = readAll(calls);
    CHECK(shape(s) == "29/4000 " && s[0].tokens[6].number == 75);
    // Without metres, or turned off: the guide's own bar (hidden)
    RicGuidance off;
    off.setBar(false);
    off.setMetresToTurn(true, 100);
    off.translate(routeStart(kLeft, "A", "100", kClusterMetres), calls, 1.0);
    CHECK(find(readAll(calls), 29, 4000).tokens[6].number == 255);
    RicGuidance unknown;
    unknown.translate(routeStart(kLeft, "A", "100", kClusterMetres), calls, 1.0);
    CHECK(find(readAll(calls), 29, 4000).tokens[6].number == 255);
    printf("PASS ric: red bar from the metres to the turn, length fixed per turn, rate-limited, off by file\n");
}

// BMW's roundabout symbol (model 11) in the place of the junction element, only with the setting.
void roundaboutSymbol() {
    CHECK(ricClockHour(0) == 0 && ricClockHour(44) == 1 && ricClockHour(90) == 3 && ricClockHour(359) == 11);
    const Seen s = read(RicCall("ric_setGuidingData", "uui(yv)", ricRoundaboutSymbol(1200, 15, 90, 2)));
    CHECK(s.id == 1200 && s.type == 11 && s.tokens[3].number == 7);  // tag 7
    // ((((1200,11),2),2), 0, 12 - 3, [0, 3, 6, 9])
    CHECK(s.tokens[4].number == 1200 && s.tokens[5].number == 11 && s.tokens[6].number == 2 && s.tokens[7].number == 2);
    CHECK(s.tokens[8].number == 0 && s.tokens[9].number == 9);
    CHECK(s.tokens.size() == 14 && s.tokens[10].number == 0 && s.tokens[13].number == 9);
    const Seen cw = read(RicCall("ric_setGuidingData", "uui(yv)", ricRoundaboutSymbol(1200, 14, 90, 2)));
    CHECK(cw.tokens[7].number == 1 && cw.tokens[9].number == 3);

    RicGuidance g;
    std::vector<RicCall> calls;
    g.setRoundaboutHint(true, 90);
    g.translate(routeStart(kRoundaboutCounterClockwise, "Sortie 1", "300", kClusterMetres), calls);
    bool symbol = false;
    for (size_t k = 0; k < calls.size(); ++k) {
        symbol = symbol || (calls[k].member == "ric_setGuidingData" && read(calls[k]).type == 11);
    }
    CHECK(!symbol);  // off by default
    RicGuidance on;
    on.setRoundaboutSymbol(true);
    on.setRoundaboutHint(true, 90);
    on.translate(routeStart(kRoundaboutCounterClockwise, "Sortie 1", "300", kClusterMetres), calls);
    std::vector<Seen> all = readAll(calls);
    CHECK(all.size() == 25 && find(all, 11, 1250).tokens[9].number == 9 && find(all, 7, 1200).type == 7);
    CHECK(find(all, 15, 200).tokens[14].number == 1250 && directionOf(find(all, 15, 200)) == 15);
    on.translate(routeStart(kRight, "A", "300", kClusterMetres), calls);  // not a roundabout: the junction
    all = readAll(calls);
    CHECK(shape(all) == std::string(kTeardown) + kStart);
    // Codes the HUD would leave blank go as the slight turn of their side
    CHECK(ricHudDirection(kLeft) == kLeft && ricHudDirection(kRoundaboutClockwise) == 14 &&
          ricHudDirection(kDestination) == 10);
    CHECK(ricHudDirection(kHighwayOutRight) == kHalfRight && ricHudDirection(kHighwayOutLeft) == kHalfLeft);
    CHECK(ricHudDirection(kLaneChangeLeft) == kHalfLeft && ricHudDirection(kForkRight) == kHalfRight);
    CHECK(ricHudDirection(0) == kStraightOn && ricHudDirection(34) == kStraightOn &&
          ricHudDirection(20) == kStraightOn);
    printf("PASS ric: roundabout symbol (model 11, clock hours, CCW mirrored), only with the setting\n");
}

// F-series: the next maneuver shows however far it is; the G20's approach hold stays the default.
void anyDistance() {
    Maneuver m;
    m.type = 7;  // a right turn
    m.road = "MAIN ST";
    m.distanceText = "8";
    m.distanceUnit = 2;  // kilometres... the unit does not matter here
    m.hasMetres = true;
    m.metres = 8000;  // 5 miles
    for (int ric = 0; ric < 2; ++ric) {
        GuideSettings settings;  // approach 1 mile
        RouteGraph graph(0xC8, 255);
        RicRecorder out;
        Guide guide(graph, out, settings);
        guide.showAtAnyDistance(ric == 1);
        guide.step(&m, 1.0);
        guide.step(&m, 2.0);
        CHECK(guide.showing() == (ric == 1) && out.ric.showing() == (ric == 1));
    }
    printf("PASS ric: a maneuver 5 miles away shown on the F-series, held back on the G20\n");
}

}  // namespace

int main() {
    everyDirection();
    junctions();
    distanceThenNewTurnThenCancel();
    textsCut();
    roundabouts();
    flipFlop();
    wholeDrive();
    anyDistance();
    bar();
    roundaboutSymbol();
    return 0;
}
