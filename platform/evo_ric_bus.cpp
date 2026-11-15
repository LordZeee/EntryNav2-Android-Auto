// Copyright (c) 2026 Jiraiya43
// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Sends the F-series arrows' RIC calls to BMW.RueKoIC on the session bus (libdbus, a private connection).
#include "evo_ric_bus.hpp"

#include <dbus/dbus.h>
#include <stdio.h>
#include <string.h>

namespace evo {
namespace kombi {

namespace {
const char* const kSessionBus = "unix:path=/tmp/session_bus_socket";
const int kTimeoutMs = 1500;
std::string g_bus;  // empty on the car: its session bus
DBusConnection* g_conn = nullptr;
uint32_t g_serial = 1;

void closeBus() {
    if (!g_conn) {
        return;
    }
    dbus_connection_close(g_conn);
    dbus_connection_unref(g_conn);
    g_conn = nullptr;
}

bool openBus() {
    static bool threads = (dbus_threads_init_default(), true);  // libdbus is also used by other threads
    (void)threads;
    if (g_conn && dbus_connection_get_is_connected(g_conn)) {
        return true;
    }
    closeBus();
    DBusError e;
    dbus_error_init(&e);
    g_conn = dbus_connection_open_private(g_bus.empty() ? kSessionBus : g_bus.c_str(), &e);
    if (g_conn && !dbus_bus_register(g_conn, &e)) {
        closeBus();
    }
    dbus_error_free(&e);
    if (!g_conn) {
        return false;
    }
    dbus_connection_set_exit_on_disconnect(g_conn, FALSE);
    return true;
}

void say(std::string* why, const std::string& text) {
    if (why) {
        *why = text;
    }
}

bool linkError(const char* name) {
    return !strcmp(name, DBUS_ERROR_SERVICE_UNKNOWN) || !strcmp(name, DBUS_ERROR_NAME_HAS_NO_OWNER) ||
           !strcmp(name, DBUS_ERROR_NO_REPLY) || !strcmp(name, DBUS_ERROR_DISCONNECTED) ||
           !strcmp(name, DBUS_ERROR_TIMEOUT);
}
}  // namespace

void setRicBusForTest(const char* address) {
    closeBus();
    g_bus = address ? address : "";
}

bool ricBusAvailable() {
    if (!openBus()) {
        return false;
    }
    DBusError e;
    dbus_error_init(&e);
    const bool has = dbus_bus_name_has_owner(g_conn, kRicDestination, &e);
    dbus_error_free(&e);
    return has;
}

SendOutcome ricBusSend(const std::vector<RicCall>& calls, std::string* why) {
    if (!openBus()) {
        say(why, "no session bus");
        return kLinkDown;
    }
    for (size_t i = 0; i < calls.size(); ++i) {
        const RicCall& c = calls[i];
        const Bytes wire = ricWireMessage(c, g_serial);
        g_serial = g_serial == 0x7FFFFFFF ? 1 : g_serial + 1;
        DBusError e;
        dbus_error_init(&e);
        DBusMessage* m =
            dbus_message_demarshal(reinterpret_cast<const char*>(&wire[0]), static_cast<int>(wire.size()), &e);
        if (!m) {
            say(why, std::string("cannot build ") + c.member + ": " + (e.message ? e.message : "?"));
            dbus_error_free(&e);
            return kRefused;
        }
        DBusMessage* r = dbus_connection_send_with_reply_and_block(g_conn, m, kTimeoutMs, &e);
        dbus_message_unref(m);
        if (!r) {
            const bool down = e.name && linkError(e.name);
            say(why, c.member + ": " + (e.name ? e.name : "no answer"));
            dbus_error_free(&e);
            if (!dbus_connection_get_is_connected(g_conn)) {
                closeBus();
            }
            return down ? kLinkDown : kRefused;
        }
        dbus_error_free(&e);
        int32_t answer = -1;
        DBusMessageIter it;
        if (dbus_message_iter_init(r, &it) && dbus_message_iter_get_arg_type(&it) == DBUS_TYPE_INT32) {
            dbus_message_iter_get_basic(&it, &answer);
        }
        dbus_message_unref(r);
        if (answer != 0 && !(c.unknownOk && answer == kRicUnknownResource)) {
            char text[160];
            snprintf(text, sizeof(text), "BMW.RueKoIC answered %d to %s (call %u of %u)", static_cast<int>(answer),
                     c.member.c_str(), static_cast<unsigned>(i + 1), static_cast<unsigned>(calls.size()));
            say(why, text);
            return kRefused;
        }
    }
    return kSent;
}

const RicBus kSessionRicBus = {ricBusAvailable, ricBusSend};

}  // namespace kombi
}  // namespace evo
