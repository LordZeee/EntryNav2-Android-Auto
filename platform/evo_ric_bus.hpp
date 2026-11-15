// Copyright (c) 2026 Jiraiya43
// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// The F-series arrows' calls to BMW.RueKoIC on the head unit's session bus, one at a time, each answer checked.

// Used by the arrows (platform/kombi_nav.cpp) through kSessionRicBus. Every call waits for its answer, as the
// factory's own client does: 0 is taken, kRicUnknownResource is taken for a dispose, anything else is a refusal.
#ifndef ENTRY_AAD_PLATFORM_EVO_RIC_BUS_HPP
#define ENTRY_AAD_PLATFORM_EVO_RIC_BUS_HPP

#include <string>
#include <vector>

#include "kombi_nav.hpp"

namespace evo {
namespace kombi {

// BMW.RueKoIC has an owner on the bus. May block up to 1.5 s.
bool ricBusAvailable();
// The calls in order; stops at the first one not taken. kLinkDown: no bus or no BMW.RueKoIC; kRefused: an answer
// other than 0, or an error reply.
SendOutcome ricBusSend(const std::vector<RicCall>& calls, std::string* why);
// Host tests: another D-Bus address than the car's session bus.
void setRicBusForTest(const char* address);

extern const RicBus kSessionRicBus;  // {ricBusAvailable, ricBusSend}

}  // namespace kombi
}  // namespace evo

#endif
