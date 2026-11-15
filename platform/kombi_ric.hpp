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
    RicManeuver() : direction(1), unit(0), bar(255) {}
    bool sameTurn(const RicManeuver& o) const { return direction == o.direction && road == o.road; }
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

class RicGuidance {
public:
    RicGuidance() : showing_(false) {}

    // The calls for one batch of the guide's requests; none for what changes nothing on the cluster (a route
    // position, the same distance again).
    void translate(const std::vector<Request>& requests, std::vector<RicCall>& out);
    // The factory teardown, whatever is shown (also after a crash, when an arrow may have been left).
    void teardown(std::vector<RicCall>& out);
    // The BMW navigation takes over: it tears our models down itself on its route start. Nothing to send.
    void forget() { showing_ = false; }
    // A batch failed half-way: something may be up, so the next change starts with a teardown.
    void assumeShown() { showing_ = true; }

    bool showing() const { return showing_; }
    const RicManeuver& shown() const { return shown_; }

private:
    void start(const RicManeuver& m, std::vector<RicCall>& out);

    bool showing_;
    RicManeuver shown_;
};

// A whole method call to BMW.RueKoIC in the D-Bus wire format, with this serial and no sender (the bus adds it).
Bytes ricWireMessage(const RicCall& call, uint32_t serial);

// The first maxChars UTF-8 characters of s, never half a character.
std::string ricCut(const std::string& s, size_t maxChars);

}  // namespace kombi
}  // namespace evo

#endif
