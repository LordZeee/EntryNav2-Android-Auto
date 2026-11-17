// Copyright (c) 2026 Jiraiya43
// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// ric-hudtest: shows one HUD sign at a time on an F-series car, with the service's own RIC code (kombi_ric.cpp).

// Run by hand over SSH, parked, with no route in BMW's navigation and none in Android Auto (both would draw too).
// Each sign stays until Enter; "q" or Ctrl-C clears the HUD with the factory teardown and quits. Not installed.
//   ric-hudtest clear                              the factory teardown, nothing else
//   ric-hudtest codes <first> <last>               direction codes one by one (road line "CODE n")
//   ric-hudtest roundabout <cw|ccw> <angle>...     BMW's roundabout symbol (model 11), one exit angle at a time
//   ric-hudtest junction <angle>...                the junction drawn with its exit at <angle> (clockwise, degrees)
//   ric-hudtest bar <metres> <seconds>             a right turn counted down from <metres> to 0: the red bar
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <string>
#include <vector>

#include "evo_ric_bus.hpp"
#include "kombi_display.hpp"
#include "kombi_graph.hpp"
#include "kombi_ric.hpp"

using namespace evo::kombi;

namespace {

volatile sig_atomic_t g_stop = 0;
void onSignal(int) {
    g_stop = 1;
}

RicGuidance g_ric;
RouteGraph g_graph(0xC8, 255);

double now() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}

bool send(const std::vector<RicCall>& calls, const char* what) {
    if (calls.empty()) {
        return true;
    }
    std::string why;
    const SendOutcome o = ricBusSend(calls, &why);
    printf("  %s: %u call(s) %s%s%s\n", what, static_cast<unsigned>(calls.size()), o == kSent ? "OK" : "FAILED",
           o == kSent ? "" : ": ", o == kSent ? "" : why.c_str());
    if (o != kSent) {
        g_ric.assumeShown();
    }
    return o == kSent;
}

void clear() {
    std::vector<RicCall> calls;
    g_ric.teardown(calls);
    send(calls, "teardown");
}

bool show(int direction, const std::string& road, const std::string& distance, int unit) {
    std::vector<Request> r;
    std::vector<RicCall> calls;
    if (!g_graph.routeStart(ClusterManeuver(direction, road, distance, unit), r)) {
        return false;
    }
    g_ric.translate(r, calls);
    return send(calls, "start");
}

// Enter: next. "q", end of input or a signal: stop.
bool next(const char* prompt) {
    printf(">> %s  [Enter = next, q = quit]\n", prompt);
    fflush(stdout);
    char line[64];
    if (g_stop || !fgets(line, sizeof(line), stdin) || g_stop) {
        return false;
    }
    return line[0] != 'q';
}

int codes(int first, int last) {
    for (int c = first; c <= last && !g_stop; ++c) {
        char road[24], prompt[96];
        snprintf(road, sizeof(road), "CODE %d", c);
        if (!show(c, road, "100", kClusterMetres)) {
            return 1;
        }
        snprintf(prompt, sizeof(prompt), "code %d shown: describe the arrow, then Enter", c);
        if (!next(prompt)) {
            break;
        }
    }
    return 0;
}

int roundabouts(bool ccw, int argc, char** argv) {
    g_ric.setRoundaboutSymbol(true);
    for (int i = 0; i < argc && !g_stop; ++i) {
        const int angle = atoi(argv[i]);
        char road[24], prompt[120];
        snprintf(road, sizeof(road), "RB %s %d", ccw ? "CCW" : "CW", angle);
        g_ric.setRoundaboutHint(true, angle);
        if (!show(ccw ? kRoundaboutCounterClockwise : kRoundaboutClockwise, road, "100", kClusterMetres)) {
            return 1;
        }
        snprintf(prompt, sizeof(prompt), "roundabout %s, exit angle %d (clock hour %d): describe it, then Enter",
                 ccw ? "counter-clockwise" : "clockwise", angle, ricClockHour(angle));
        if (!next(prompt)) {
            break;
        }
    }
    return 0;
}

int junctions(int argc, char** argv) {
    // A direction whose own angle is the one asked: the exit road is ricExitAngle(direction); others go through the
    // roundabout hint, which turns any angle into an exit road.
    for (int i = 0; i < argc && !g_stop; ++i) {
        const int angle = atoi(argv[i]);
        char road[24], prompt[96];
        snprintf(road, sizeof(road), "JUNCTION %d", angle);
        g_ric.setRoundaboutSymbol(false);
        g_ric.setRoundaboutHint(true, 180 - angle);  // counter-clockwise roundabout: exit road at 180 - hint
        if (!show(kRoundaboutCounterClockwise, road, "100", kClusterMetres)) {
            return 1;
        }
        snprintf(prompt, sizeof(prompt), "junction with its exit at %d degrees: describe it, then Enter", angle);
        if (!next(prompt)) {
            break;
        }
    }
    return 0;
}

int bar(double from, double seconds) {
    g_ric.setMetresToTurn(true, from);
    char text[16];
    snprintf(text, sizeof(text), "%.0f", from);
    if (!show(kRight, "BAR TEST", text, kClusterMetres)) {
        return 1;
    }
    printf("  counting down %.0f m over %.0f s: watch the red bar\n", from, seconds);
    const double t0 = now();
    std::string lastText = text;
    while (!g_stop) {
        const double t = now() - t0;
        const double metres = t >= seconds ? 0 : from * (1 - t / seconds);
        g_ric.setMetresToTurn(true, metres);
        snprintf(text, sizeof(text), "%.0f", metres >= 50 ? (static_cast<int>(metres) / 50) * 50.0 : metres);
        std::vector<RicCall> calls;
        if (text != lastText) {
            std::vector<Request> r;
            g_graph.distanceUpdate(text, kClusterMetres, r);
            g_ric.translate(r, calls, now());
            lastText = text;
        } else {
            g_ric.refreshBar(now(), calls);
        }
        if (!calls.empty()) {
            printf("  %5.0f m, bar %3d%%\n", metres, ricBar(from < kRicBarShowM ? from : kRicBarShowM, metres));
            if (!send(calls, "distance")) {
                return 1;
            }
        }
        if (t >= seconds + 3) {
            break;
        }
        struct timespec ts = {0, 200000000};
        nanosleep(&ts, nullptr);
    }
    next("countdown over: describe the bar, then Enter");
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = onSignal;  // no SA_RESTART: a signal ends the wait for Enter
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);
    setvbuf(stdout, nullptr, _IOLBF, 0);

    if (argc < 2) {
        fprintf(stderr,
                "usage: %s clear | codes <first> <last> | roundabout <cw|ccw> <angle>... | junction <angle>... |"
                " bar <metres> <seconds>\n",
                argv[0]);
        return 2;
    }
    if (!ricBusAvailable()) {
        fprintf(stderr, "BMW.RueKoIC is not on the session bus: nothing sent\n");
        return 1;
    }
    const std::string mode = argv[1];
    int rc = 2;
    if (mode == "clear") {
        g_ric.assumeShown();
        clear();
        return 0;
    } else if (mode == "codes" && argc == 4) {
        rc = codes(atoi(argv[2]), atoi(argv[3]));
    } else if (mode == "roundabout" && argc >= 4 && (!strcmp(argv[2], "cw") || !strcmp(argv[2], "ccw"))) {
        rc = roundabouts(!strcmp(argv[2], "ccw"), argc - 3, argv + 3);
    } else if (mode == "junction" && argc >= 3) {
        rc = junctions(argc - 2, argv + 2);
    } else if (mode == "bar" && argc == 4) {
        rc = bar(atof(argv[2]), atof(argv[3]));
    } else {
        fprintf(stderr, "bad arguments (run without any for the usage)\n");
        return 2;
    }
    if (g_ric.showing()) {
        clear();
    }
    printf("done%s\n", g_stop ? " (stopped)" : "");
    return rc;
}
