// Copyright (c) 2026 Jiraiya43
// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// The F-series RIC calls: reading the guide's G20 requests, the factory's sequences, the D-Bus wire format.
#include "kombi_ric.hpp"

#include <math.h>
#include <string.h>

#include "kombi_ric_template.hpp"

namespace evo {
namespace kombi {

const char* const kRicDestination = "BMW.RueKoIC";
const char* const kRicPath = "/BMW/RueKoIC";
const char* const kRicInterface = "BMW.INFOTAINMENT.REMOTEINSTRUMENTCLUSTER.RICServiceGuidance";

namespace {

// The G20 request bodies, as kombi_graph.cpp builds them.
const size_t kModelIdAt = 4;
const size_t kModelTypeAt = 8;
const size_t kTypeEchoAt = 21;
const size_t kBarAt = 22;
const size_t kStateAt = 30;
const size_t kDirectionAt = 33;
const size_t kTextFrom = 18;
const uint16_t kTextMethod = 0x0003;
const uint16_t kRouteMethod = 0x0009;
const uint16_t kManeuverMethod = 0x000A;
const uint8_t kTextModel = 3, kDescriptionModel = 15, kDistanceModel = 29, kStateModel = 33;

// The F-series model ids and values, as the factory uses them (one maneuver: 200).
const uint32_t kInstance = 1;
const uint32_t kManeuverId = 200;
const uint32_t kRoadTextId = 9000;
const uint32_t kDistanceId = 4000;
const uint32_t kStateId = 2, kCarInfoId = 3, kListId = 1;
const uint32_t kNone = 0xFFFFFFFFu;
const uint64_t kNoRoute = 0x80000000ull;  // the state model's route field before a route is known
const double kRoutePosition = 136509.0;   // maneuver 0's route position in the template (metres to destination)
const uint16_t kHeading = 224;
const char* const kModelSig = "uui(yv)";

// D-Bus wire format, little-endian.
class Wire {
public:
    Bytes b;
    void pad(size_t n) {
        while (b.size() % n) {
            b.push_back(0);
        }
    }
    void u8(uint8_t v) { b.push_back(v); }
    void u16(uint16_t v) {
        pad(2);
        put(&v, 2);
    }
    void u32(uint32_t v) {
        pad(4);
        put(&v, 4);
    }
    void u64(uint64_t v) {
        pad(8);
        put(&v, 8);
    }
    void f64(double v) {
        pad(8);
        put(&v, 8);
    }
    void str(const std::string& s) {
        u32(static_cast<uint32_t>(s.size()));
        b.insert(b.end(), s.begin(), s.end());
        b.push_back(0);
    }
    void sig(const char* s) {
        const size_t n = strlen(s);
        b.push_back(static_cast<uint8_t>(n));
        b.insert(b.end(), s, s + n);
        b.push_back(0);
    }
    // ric_setXxx(1, id, type, (tag, <variantSig>...)): everything up to the model's own fields.
    void modelHead(uint32_t id, uint32_t type, uint8_t tag, const char* variantSig) {
        u32(kInstance);
        u32(id);
        u32(type);
        pad(8);
        u8(tag);
        sig(variantSig);
        pad(8);
    }

private:
    void put(const void* p, size_t n) {
        const uint8_t* c = static_cast<const uint8_t*>(p);
        b.insert(b.end(), c, c + n);  // the head unit and every host here are little-endian
    }
};

uint32_t be32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) << 24 | static_cast<uint32_t>(p[1]) << 16 | static_cast<uint32_t>(p[2]) << 8 |
           p[3];
}

void appendUtf8(std::string& out, uint32_t c) {
    if (c < 0x80) {
        out += static_cast<char>(c);
    } else if (c < 0x800) {
        out += static_cast<char>(0xC0 | c >> 6);
        out += static_cast<char>(0x80 | (c & 0x3F));
    } else if (c < 0x10000) {
        out += static_cast<char>(0xE0 | c >> 12);
        out += static_cast<char>(0x80 | (c >> 6 & 0x3F));
        out += static_cast<char>(0x80 | (c & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | c >> 18);
        out += static_cast<char>(0x80 | (c >> 12 & 0x3F));
        out += static_cast<char>(0x80 | (c >> 6 & 0x3F));
        out += static_cast<char>(0x80 | (c & 0x3F));
    }
}

// The UTF-16BE text after the first byte-order mark from `from`; *after: the index after its terminator.
bool utf16Text(const Bytes& b, size_t from, std::string& out, size_t* after) {
    size_t i = from;
    while (i + 1 < b.size() && !(b[i] == 0xFE && b[i + 1] == 0xFF)) {
        ++i;
    }
    if (i + 1 >= b.size()) {
        return false;
    }
    out.clear();
    for (i += 2; i + 1 < b.size();) {
        uint32_t c = static_cast<uint32_t>(b[i]) << 8 | b[i + 1];
        i += 2;
        if (c == 0) {
            break;
        }
        if (c >= 0xD800 && c < 0xDC00 && i + 1 < b.size()) {
            const uint32_t low = static_cast<uint32_t>(b[i]) << 8 | b[i + 1];
            i += 2;
            c = 0x10000 + ((c - 0xD800) << 10) + (low - 0xDC00);
        }
        appendUtf8(out, c);
    }
    if (after) {
        *after = i;
    }
    return true;
}

RicCall templated(const rictmpl::Call& c) {
    return RicCall(c.member, kModelSig, Bytes(c.body, c.body + c.len));
}

// Guidance state (33): ((2,33), active maneuver, -1, state, route, 0, distance, -1)
RicCall stateCall(uint32_t active, int32_t state) {
    Wire w;
    w.modelHead(kStateId, kStateModel, 4, "((ui)uuittdu)");
    w.u32(kStateId);
    w.u32(kStateModel);
    w.u32(active);
    w.u32(kNone);
    w.u32(static_cast<uint32_t>(state));
    w.u64(kNoRoute);
    w.u64(0);
    w.f64(0.0);
    w.u32(kNone);
    return RicCall("ric_setManeuverCtrlData", kModelSig, w.b);
}

// Car info (17): ((3,17), distance, heading, text, 0, (255, 255, (0, 0, [])), 0, 0, a, b)
RicCall carInfoCall(bool zero) {
    Wire w;
    w.modelHead(kCarInfoId, 17, 5, "((ui)dqsi(yy(ddai))uqqq)");
    w.u32(kCarInfoId);
    w.u32(17);
    w.f64(0.0);
    w.u16(kHeading);
    w.str(zero ? "" : "0.0");
    w.u32(0);
    w.pad(8);
    w.u8(255);
    w.u8(255);
    w.pad(8);
    w.f64(0.0);
    w.f64(0.0);
    w.u32(0);  // empty array of int
    w.u32(0);
    w.u16(0);
    w.u16(zero ? 0 : 80);
    w.u16(zero ? 0 : 105);
    return RicCall("ric_setManeuverCtrlData", kModelSig, w.b);
}

// Text resource (3): ((9000,3), 0, text)
RicCall roadTextCall(const std::string& road) {
    Wire w;
    w.modelHead(kRoadTextId, kTextModel, 9, "((ui)qs)");
    w.u32(kRoadTextId);
    w.u32(kTextModel);
    w.u16(0);
    w.str(road);
    return RicCall("ric_setResource", kModelSig, w.b);
}

// Distance (29): ((4000,29), bar, route position, [25, 50, 75], text, unit, 0)
RicCall distanceCall(const RicManeuver& m) {
    Wire w;
    w.modelHead(kDistanceId, kDistanceModel, 2, "((ui)ydaysit)");
    w.u32(kDistanceId);
    w.u32(kDistanceModel);
    w.u8(static_cast<uint8_t>(m.bar));
    w.f64(kRoutePosition);
    w.u32(3);
    w.u8(25);
    w.u8(50);
    w.u8(75);
    w.str(m.distance);
    w.u32(static_cast<uint32_t>(m.unit));
    w.u64(0);
    return RicCall("ric_setManeuverData", kModelSig, w.b);
}

// Description (15): (((200,15), 0, 0, -1, 0, -1), direction, road 800, thresholds 700, graph 1200, distance 4000,
// the six events, [], [])
RicCall descriptionCall(int direction) {
    Wire w;
    w.modelHead(kManeuverId, kDescriptionModel, 5, "(((ui)iiuiu)iuuuuauauau)");
    w.pad(8);
    w.u32(kManeuverId);
    w.u32(kDescriptionModel);
    w.u32(0);
    w.u32(0);
    w.u32(kNone);
    w.u32(0);
    w.u32(kNone);
    w.u32(static_cast<uint32_t>(direction));
    w.u32(800);
    w.u32(700);
    w.u32(1200);
    w.u32(kDistanceId);
    w.u32(6 * 4);
    for (uint32_t i = 0; i < 6; ++i) {
        w.u32(15000 + i);
    }
    w.u32(0);
    w.u32(0);
    return RicCall("ric_setManeuverData", kModelSig, w.b);
}

// Maneuver list (32): ((1,32), 0, n, n, ids, 0, 0, 0, [])
RicCall listCall(bool withManeuver) {
    Wire w;
    w.modelHead(kListId, 32, 1, "((ui)qqqauqqqau)");
    w.u32(kListId);
    w.u32(32);
    const uint16_t n = withManeuver ? 1 : 0;
    w.u16(0);
    w.u16(n);
    w.u16(n);
    w.u32(n * 4);
    if (withManeuver) {
        w.u32(kManeuverId);
    }
    w.u16(0);
    w.u16(0);
    w.u16(0);
    w.u32(0);
    return RicCall("ric_setManeuverListData", kModelSig, w.b);
}

RicCall initializeCall() {
    Wire w;
    w.u32(kInstance);
    return RicCall("ric_initialize", "u", w.b);
}

RicCall disposeCall(uint32_t id) {
    Wire w;
    w.u32(kInstance);
    w.u32(id);
    return RicCall("ric_disposeResource", "uu", w.b, true);
}

// The captured graph's points, as the template has them (moved to 50 N 10 E, shape kept): 1 the approach's start, 2
// the junction, 3 to 5 the side roads, 6 the exit's end.
const double kGraphLat[6] = {
    50.0, 50.000225, 50.00023681285804, 50.000330002368415, 50.00023681285804, 50.00023203676723};
const double kGraphLon[6] = {10.0, 10.0, 9.999748189950346, 9.9997, 9.9997, 10.0003};
const double kExitMetres = 25.0;
const double kMetresPerDegreeLat = 111320.0;
const double kMetresPerDegreeLon = 71555.0;  // at 50 degrees north
const double kPi = 3.14159265358979323846;

// The captured graph's elements: id, points, the y and last u fields as captured.
struct GraphElement {
    uint64_t id;
    uint64_t points[3];
    uint32_t count;
    uint32_t kind;
};
const GraphElement kGraphElements[] = {{2000, {1, 2, 2}, 3, 7},
                                       {1401, {3, 2, 2}, 3, 4},
                                       {1402, {4, 3, 3}, 3, 4},
                                       {1403, {5, 3, 3}, 3, 7},
                                       {2001, {2, 6, 0}, 2, 4}};
const uint32_t kGraphId = 1300, kGraphModel = 31;

bool isGraphCall(const RicCall& c) {
    if (c.member != "ric_setGuidingData" || c.body.size() < 12) {
        return false;
    }
    uint32_t type = 0;
    memcpy(&type, &c.body[8], 4);
    return type == kGraphModel;
}

}  // namespace

double ricExitAngle(int direction) {
    switch (direction) {
        case 2:  // kHalfLeft
            return -45;
        case 3:  // kHalfRight
            return 45;
        case 4:  // kTightLeft
            return -135;
        case 5:  // kTightRight
            return 135;
        case 6:  // kLeft
            return -90;
        case 7:  // kRight
            return 90;
        case 8:  // kUTurnLeft, kUTurnRight: drawn from the direction anyway
        case 9:
            return 180;
        case 25:  // kHighwayInLeft, kHighwayOutLeft, kForkLeft
        case 28:
        case 37:
            return -30;
        case 26:  // kHighwayInRight, kHighwayOutRight, kForkRight
        case 29:
        case 38:
            return 30;
        case 32:  // kLaneChangeLeft / Right
            return -20;
        case 33:
            return 20;
        default:  // straight on, destination, roundabouts until their own graph is known
            return 0;
    }
}

Bytes ricGraphBody(const double lat[6], const double lon[6]) {
    Wire w;
    w.modelHead(kGraphId, kGraphModel, 2, "((ui)a(tuuatydu)a(t(ddq)))");
    w.u32(kGraphId);
    w.u32(kGraphModel);
    w.u32(0);
    size_t lengthAt = w.b.size() - 4;
    w.pad(8);
    size_t from = w.b.size();
    for (size_t i = 0; i < sizeof(kGraphElements) / sizeof(kGraphElements[0]); ++i) {
        const GraphElement& e = kGraphElements[i];
        w.pad(8);
        w.u64(e.id);
        w.u32(kNone);
        w.u32(3508392);
        w.u32(e.count * 8);
        w.pad(8);
        for (uint32_t k = 0; k < e.count; ++k) {
            w.u64(e.points[k]);
        }
        w.u8(2);
        w.f64(12.0);
        w.u32(e.kind);
    }
    uint32_t length = static_cast<uint32_t>(w.b.size() - from);
    memcpy(&w.b[lengthAt], &length, 4);
    w.u32(0);
    lengthAt = w.b.size() - 4;
    w.pad(8);
    from = w.b.size();
    for (int i = 0; i < 6; ++i) {
        w.pad(8);
        w.u64(static_cast<uint64_t>(i + 1));
        w.pad(8);
        w.f64(lat[i]);
        w.f64(lon[i]);
        w.u16(0);
    }
    length = static_cast<uint32_t>(w.b.size() - from);
    memcpy(&w.b[lengthAt], &length, 4);
    return w.b;
}

Bytes ricTurnGraph(double angle) {
    double lat[6], lon[6];
    for (int i = 0; i < 6; ++i) {
        lat[i] = kGraphLat[i];
        lon[i] = kGraphLon[i];
    }
    if (angle < 0) {  // the side roads to the right of the junction, away from the exit
        for (int i = 2; i < 5; ++i) {
            lon[i] = 2 * kGraphLon[1] - kGraphLon[i];
        }
    }
    const double rad = angle * kPi / 180.0;
    lat[5] = lat[1] + kExitMetres * cos(rad) / kMetresPerDegreeLat;
    lon[5] = lon[1] + kExitMetres * sin(rad) / kMetresPerDegreeLon;
    return ricGraphBody(lat, lon);
}

std::string ricCut(const std::string& s, size_t maxChars) {
    size_t chars = 0, i = 0;
    while (i < s.size()) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        size_t len = c < 0x80 ? 1 : (c >> 5) == 6 ? 2 : (c >> 4) == 14 ? 3 : (c >> 3) == 30 ? 4 : 1;
        if (chars == maxChars || i + len > s.size()) {
            break;
        }
        i += len;
        ++chars;
    }
    return s.substr(0, i);
}

G20Intent readG20(const std::vector<Request>& requests, const RicManeuver& base) {
    G20Intent in;
    in.maneuver = base;
    bool cancelState = false;
    for (size_t k = 0; k < requests.size(); ++k) {
        const Request& r = requests[k];
        const Bytes& b = r.body;
        if (b.size() <= kModelTypeAt) {
            continue;
        }
        const uint8_t type = b[kModelTypeAt];
        if (r.method == kRouteMethod && type == kStateModel && b.size() > kStateAt) {
            const int state = b[kStateAt];
            cancelState = cancelState || state == 0 || state == 5 || state == 7;
            continue;
        }
        if (b.size() < kModelIdAt + 4 || (be32(&b[kModelIdAt]) & 0x00FFFFFF) != 0) {
            continue;  // only maneuver 0
        }
        if (r.method == kManeuverMethod && type == kDescriptionModel && b.size() > kDirectionAt &&
            b[kTypeEchoAt] == kDescriptionModel) {
            in.maneuver.direction = b[kDirectionAt];
            in.hasDirection = true;
        } else if (r.method == kManeuverMethod && type == kDistanceModel && b.size() > kBarAt &&
                   b[kTypeEchoAt] == kDistanceModel) {
            std::string text;
            size_t after = 0;
            if (utf16Text(b, kTextFrom, text, &after)) {
                in.maneuver.distance = ricCut(text, kRicDistanceChars);
                if (after < b.size()) {
                    in.maneuver.unit = b[after];
                }
            }
            in.maneuver.bar = b[kBarAt];
            in.hasDistance = true;
        } else if (r.method == kTextMethod && type == kTextModel) {
            std::string text;
            if (utf16Text(b, kTextFrom, text, nullptr)) {
                in.maneuver.road = ricCut(text, kRicRoadChars);
                in.hasRoad = true;
            }
        }
    }
    in.cancel = cancelState && !in.hasDirection;
    return in;
}

void RicGuidance::translate(const std::vector<Request>& requests, std::vector<RicCall>& out) {
    out.clear();
    const G20Intent in = readG20(requests, shown_);
    if (in.cancel) {
        if (showing_) {
            teardown(out);
        }
        return;
    }
    if (!in.hasDirection && !in.hasDistance && !in.hasRoad) {
        return;  // a route position, a lane model: nothing the F-series models carry
    }
    if (!showing_ || !in.maneuver.sameTurn(shown_)) {
        if (!in.hasDirection && !showing_) {
            return;  // a distance before any maneuver: the guide always starts with a route start
        }
        if (showing_) {
            teardown(out);
        }
        start(in.maneuver, out);
        return;
    }
    if (!in.maneuver.sameDistance(shown_)) {
        out.push_back(distanceCall(in.maneuver));
        shown_ = in.maneuver;
    }
}

void RicGuidance::start(const RicManeuver& m, std::vector<RicCall>& out) {
    out.push_back(stateCall(kNone, 1));
    out.push_back(carInfoCall(false));
    out.push_back(roadTextCall(m.road.empty() ? std::string(" ") : m.road));
    for (size_t i = 0; i < sizeof(rictmpl::before_distance) / sizeof(rictmpl::before_distance[0]); ++i) {
        out.push_back(templated(rictmpl::before_distance[i]));
    }
    out.push_back(distanceCall(m));
    for (size_t i = 0; i < sizeof(rictmpl::after_distance) / sizeof(rictmpl::after_distance[0]); ++i) {
        out.push_back(templated(rictmpl::after_distance[i]));
        if (isGraphCall(out.back())) {
            out.back().body = ricTurnGraph(ricExitAngle(m.direction));  // the arrow is this junction's exit
        }
    }
    out.push_back(descriptionCall(m.direction));
    out.push_back(listCall(true));
    out.push_back(initializeCall());
    out.push_back(stateCall(kManeuverId, 2));
    showing_ = true;
    shown_ = m;
}

void RicGuidance::teardown(std::vector<RicCall>& out) {
    out.push_back(stateCall(kManeuverId, 5));
    out.push_back(disposeCall(kRoadTextId));
    out.push_back(stateCall(kNone, 0));
    out.push_back(listCall(false));
    out.push_back(carInfoCall(true));
    out.push_back(initializeCall());
    showing_ = false;
}

Bytes ricWireMessage(const RicCall& call, uint32_t serial) {
    Wire w;
    w.u8('l');
    w.u8(1);  // method call
    w.u8(0);
    w.u8(1);
    w.u32(static_cast<uint32_t>(call.body.size()));
    w.u32(serial);
    const size_t lengthAt = w.b.size();
    w.u32(0);
    const size_t fieldsFrom = w.b.size();
    struct Field {
        uint8_t code;
        const char* type;
        const char* value;
    };
    const Field fields[] = {{1, "o", kRicPath},
                            {2, "s", kRicInterface},
                            {3, "s", call.member.c_str()},
                            {6, "s", kRicDestination},
                            {8, "g", call.signature.c_str()}};
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        w.pad(8);
        w.u8(fields[i].code);
        w.sig(fields[i].type);
        if (fields[i].type[0] == 'g') {
            w.sig(fields[i].value);
        } else {
            w.str(fields[i].value);
        }
    }
    const uint32_t fieldsLength = static_cast<uint32_t>(w.b.size() - fieldsFrom);
    memcpy(&w.b[lengthAt], &fieldsLength, 4);
    w.pad(8);
    w.b.insert(w.b.end(), call.body.begin(), call.body.end());
    return w.b;
}

}  // namespace kombi
}  // namespace evo
