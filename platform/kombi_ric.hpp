// Copyright (c) 2026 Jiraiya43
// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// The arrows on F-series cars: the guide's G20 requests turned into the F-series RIC calls on D-Bus. No I/O.

// On an F48 there is no cluster on the car's Ethernet: the head unit's own navigation draws the HUD and cluster arrows
// by calling the RIC guidance interface of BMW.RueKoIC (/BMW/RueKoIC, a part of NavStartup) on the session bus, with
// the same models and values as the G20's BMW.IC.Guidance. RicGuidance reads what the guide put in maneuver 0
// (direction, distance text and unit, red bar, road) out of the requests built for the G20 (kombi_graph.hpp), and
// writes the calls the factory makes for the same picture, in the factory's order (captured on an X1 F48):
//   start     guidance state 1, car info, road text, road data, distance, thresholds, graph and order, six
//             sub-distances each with its info event, description, maneuver list [200], initialize,
//             guidance state 2 with maneuver 200 active
//   teardown  guidance state 5, the road text disposed, guidance state 0, empty list, car info zero, initialize
//   distance  the distance model of maneuver 0 alone
// A new direction or road is a teardown and a new start, as the factory does on a new route: nothing is ever changed
// under a maneuver the cluster is showing. Texts are cut to what the factory sends.
#ifndef ENTRY_AAD_PLATFORM_KOMBI_RIC_HPP
#define ENTRY_AAD_PLATFORM_KOMBI_RIC_HPP

#include <stdint.h>

#include <string>
#include <vector>

#include "kombi_display.hpp"
#include "kombi_someip.hpp"

namespace evo {
namespace kombi {

extern const char* const kRicDestination;  // BMW.RueKoIC
extern const char* const kRicPath;         // /BMW/RueKoIC
extern const char* const kRicInterface;    // BMW.INFOTAINMENT.REMOTEINSTRUMENTCLUSTER.RICServiceGuidance
const size_t kRicRoadChars = 20;           // the factory's road names were 3 to 13 characters
const size_t kRicDistanceChars = 6;
const int32_t kRicUnknownResource = 48;  // the answer to disposing a text that isn't there: harmless

// One method call to BMW.RueKoIC: its member, signature and body in the D-Bus wire format (little-endian).
struct RicCall {
    std::string member;
    std::string signature;
    Bytes body;
    bool unknownOk;  // kRicUnknownResource is an acceptable answer (dispose)
    RicCall() : unknownOk(false) {}
    RicCall(const char* m, const char* s, const Bytes& b, bool u = false)
        : member(m), signature(s), body(b), unknownOk(u) {}
};

// Maneuver 0 as the F-series models carry it.
struct RicManeuver {
    int direction;         // Direction (kombi_display.hpp)
    std::string distance;  // as the guide wrote it, cut to kRicDistanceChars
    int unit;              // ClusterUnit
    int bar;               // the red bar's percentage, 255 hides it
    std::string road;      // cut to kRicRoadChars
    double angle;          // the junction's exit road, clockwise from straight on (ricExitAngle, or a roundabout's)
    RicManeuver() : direction(1), unit(0), bar(255), angle(0) {}
    bool sameTurn(const RicManeuver& o) const { return direction == o.direction && road == o.road && angle == o.angle; }
    bool sameDistance(const RicManeuver& o) const { return distance == o.distance && unit == o.unit && bar == o.bar; }
};

// What one batch of G20 requests asks for.
struct G20Intent {
    bool cancel;           // guidance state 0, 5 or 7 without a maneuver: the factory cancel
    bool hasDirection;     // maneuver 0's description
    bool hasDistance;      // maneuver 0's distance
    bool hasRoad;          // maneuver 0's road text
    RicManeuver maneuver;  // what was found, the rest from `base`
    G20Intent() : cancel(false), hasDirection(false), hasDistance(false), hasRoad(false) {}
};
// Reads the batch on top of `base` (the maneuver last shown).
G20Intent readG20(const std::vector<Request>& requests, const RicManeuver& base);

// A new turn shown less than this after the last one waits (the phone flips between two maneuvers while the car
// stands at a junction, about once a second); it is dropped if the phone flips back meanwhile.
const double kRicTurnHoldS = 1.5;

// The red bar (BMW's RIC client and BMW.RueKoIC, read in their code): fill = 100 (L - D) / L with D the metres to the
// turn and L fixed when the turn becomes the current one, both less a 9 m rounding offset; L is the smaller of D then
// and kRicBarShowM, where BMW enters guiding step 3, the only one with a bar. Hidden (255) while D > L. The HUD draws
// 100 - fill, so 100 is an empty bar (seen on an X1 F48 with 100).
const double kRicBarShowM = 170.0;
const double kRicBarOffsetM = 9.0;
const double kRicBarEveryS = 0.5;       // bar-only distance updates at most this often
const double kRicLanesEveryS = 1.0;     // lane model updates at most this often
const double kRicLanePreviewM = 300.0;  // lane model preview distance and range
int ricBar(double length, double metres);

class RicGuidance {
public:
    RicGuidance()
        : showing_(false),
          shownAt_(-1e9),
          pending_(false),
          hint_(false),
          hintAngle_(0),
          barOn_(true),
          roundaboutSymbol_(false),
          metresKnown_(false),
          metres_(0),
          barLength_(0),
          barAt_(-1e9),
          lanesOn_(false),
          laneSlot_(1),
          lanesAt_(-1e9),
          flagOn_(false),
          flagHigh_(false),
          detail_(0),
          hudDirection_(1) {}

    // The phone's metres to the maneuver being guided, for the bar; none without. The G20 requests do not carry them.
    void setMetresToTurn(bool known, double metres) {
        metresKnown_ = known;
        metres_ = metres;
    }
    void setBar(bool on) { barOn_ = on; }
    // Roundabouts with the phone's exit angle as BMW's roundabout symbol (model 11). Off: an angled junction.
    void setRoundaboutSymbol(bool on) { roundaboutSymbol_ = on; }
    // The phone's lanes for the maneuver being guided (empty: none; phones send them near the junction only).
    void setLanes(const ClusterLanes& lanes) { lanes_ = lanes; }
    // The lane row as BMW's lane model (27). laneSlot: which of the description's two last lists links it (1 or 2).
    void setLaneGuidance(bool on, int laneSlot = 1) {
        lanesOn_ = on;
        laneSlot_ = laneSlot;
    }
    // The destination as BMW's flag: model 5, or 49 with high.
    void setDestinationFlag(bool on, bool high = false) {
        flagOn_ = on;
        flagHigh_ = high;
    }
    // On every tick of the service: the lane model when the lanes changed (at most every kRicLanesEveryS), a
    // distance call when the bar moved (at most every kRicBarEveryS).
    void refresh(double now, std::vector<RicCall>& out);

    // The calls for one batch of the guide's requests; none for what changes nothing on the cluster (a route
    // position, the same distance again, a turn held back). now: seconds of a monotonic clock; negative, no hold.
    void translate(const std::vector<Request>& requests, std::vector<RicCall>& out, double now = -1);
    // A turn held back long enough: its teardown and start. Called on every tick of the service.
    void flush(double now, std::vector<RicCall>& out);
    // The factory teardown, whatever is shown (also after a crash, when an arrow may have been left).
    void teardown(std::vector<RicCall>& out);
    // The BMW navigation takes over: it tears our models down itself on its route start. Nothing to send.
    void forget() {
        showing_ = false;
        pending_ = false;
    }
    // A batch failed half-way: something may be up, so the next change starts with a teardown.
    void assumeShown() { showing_ = true; }
    // The phone's roundabout exit angle for the maneuver being guided (Android Auto's, 1 to 360 degrees around the
    // roundabout from its entry); none for any other maneuver. The G20 requests do not carry it.
    void setRoundaboutHint(bool known, int exitAngle) {
        hint_ = known;
        hintAngle_ = exitAngle;
    }

    bool showing() const { return showing_; }
    bool pending() const { return pending_; }
    const RicManeuver& shown() const { return shown_; }

private:
    void start(const RicManeuver& m, std::vector<RicCall>& out);
    double angleFor(const RicManeuver& m) const;
    double barLengthNow() const { return metres_ < kRicBarShowM ? metres_ : kRicBarShowM; }
    void applyBar(RicManeuver& m) const;

    bool showing_;
    RicManeuver shown_;
    double shownAt_;
    bool pending_;
    RicManeuver pendingTurn_;
    bool hint_;
    int hintAngle_;
    bool barOn_;
    bool roundaboutSymbol_;
    bool metresKnown_;
    double metres_;
    double barLength_;  // L of the turn shown
    double barAt_;
    bool lanesOn_;
    int laneSlot_;
    ClusterLanes lanes_;
    ClusterLanes lanesShown_;
    double lanesAt_;
    bool flagOn_;
    bool flagHigh_;
    uint32_t detail_;   // the description's detailed maneuver id at the last start
    int hudDirection_;  // its direction code
};

// A roundabout's exit as the junction's exit road: the arc driven around it is the phone's exit angle, so the road
// leaves at 180 - angle (right-hand traffic, counter-clockwise, direction 15) or angle - 180 (clockwise, 14), kept
// within +/-160 degrees.
double ricRoundaboutTurn(int direction, int exitAngle);
// BMW's clock position (0 to 11) for an angle in degrees: hour = T[angle / 22].
int ricClockHour(double angle);
// BMW's roundabout symbol (ric_setGuidingData, model 11, tag 7): ((((id,11), maneuver type), drive direction 1 CW /
// 2 CCW), entry 0, exit sector, [the roads' sectors]): 30-degree sectors counted in the driving direction from the
// entry, either direction (the HUD mirrors them). Field meanings from BMW's code.
Bytes ricRoundaboutSymbol(uint32_t id, int direction, int exitAngle, int32_t maneuverType);
// BMW's lane model (ric_setLaneGuidingData, model 27, tag 1): one descriptor per lane, left to right (existing lane;
// best lane for the ones to take, else possible; no change hint; its arrow), solid edges and dashed markers between,
// the lane point at the maneuver's route position, preview and range kRicLanePreviewM, not toll lanes.
Bytes ricLaneModel(uint32_t id, const ClusterLanes& lanes);
// BMW's destination flag: DestinationReachedManeuver (model 5, tag 10) or, high, ManeuverHighDestReachedElement
// (model 49, tag 1, no picture); maneuver type 0, stop-over 0 (the final destination).
Bytes ricDestinationFlag(uint32_t id, bool high);
// The exit number on the road line ("Exit 2 - ..."), 0 without one.
int ricExitNumber(const std::string& road);

// A whole method call to BMW.RueKoIC in the D-Bus wire format, with this serial and no sender (the bus adds it).
Bytes ricWireMessage(const RicCall& call, uint32_t serial);

// The F-series HUD draws the arrow from the guiding graph (model 31), not from the direction (checked on an X1 F48:
// every direction showed the captured junction, except the U-turns 8 and 9). So each maneuver gets the captured
// junction with its exit road turned: the angle, clockwise from straight on (90 right, -90 left, 180 U-turn).
double ricExitAngle(int direction);
// The direction code sent in the description: BMW.RueKoIC draws only 1 to 9, 14, 15 (and 10 to 12, 37 to 41 as
// straight or slight turns); any other code leaves the HUD blank. Motorway, lane-change and fork codes go as the
// slight turn of their side, 0 and the rest as straight on.
int ricHudDirection(int direction);
// The graph's body (uui(yv), model 31): the captured elements, the six points given (point 2 is the junction, point 6
// the exit's end). With the template's own points it is the captured graph byte for byte.
Bytes ricGraphBody(const double lat[6], const double lon[6]);
// The captured junction with the exit road at `angle`; the side roads go to the other side of a left turn.
Bytes ricTurnGraph(double angle);

// The first maxChars UTF-8 characters of s, never half a character.
std::string ricCut(const std::string& s, size_t maxChars);

}  // namespace kombi
}  // namespace evo

#endif
