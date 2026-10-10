#pragma once
// Which pilot sets the kinematic drone's velocity this cycle: the gamepad, a program calling
// DroneControl, or nobody. Header-only and free of Webots so it is unit tested without a
// simulator (tests/test_drone_command.cpp).
//
// ★ THE GAMEPAD ALWAYS WINS, BUT ONLY WHILE SOMEONE IS ACTUALLY HOLDING A STICK. A human in drone
//   mode is the safety pilot: any off-centre stick takes over from the program at once. A centred
//   stick does not, so a pad left in drone mode does not ground a program that is flying.
// ★ BOTH SOURCES EXPIRE. A program that stops calling setVelocity (crashed, hung, network gone)
//   must not leave the drone cruising into the geofence, and neither must a pad that stopped
//   publishing with a stick leaning. After watchdog_s without news the target is zero: hover.
#include <algorithm>
#include <cmath>

namespace drone
{
    // Drone BODY frame: forward = the drone's nose, lateral = its right, climb = up,
    // yaw = CCW about +z. SI units (m/s, rad/s) unless stated otherwise.
    struct Velocity { double forward = 0, lateral = 0, climb = 0, yaw = 0; };

    // Speeds at full stick, and the ceiling a program may ask for.
    struct Limits { double linear = 1.0, climb = 0.7, yaw = 1.5; };

    enum class Source { Stick, Program, Hover };

    struct Selection { Velocity target; Source source = Source::Hover; };

    inline const char *to_string(Source s)
    {
        switch (s)
        {
            case Source::Stick:   return "STICK";
            case Source::Program: return "PROGRAM";
            case Source::Hover:   return "HOVER";
        }
        return "?";
    }

    // stick: normalised to [-1, 1]. api: SI units. *_age_s: seconds since the last message of
    // that kind (a huge value means "never").
    inline Selection select_target(bool drone_mode, const Velocity &stick, double stick_age_s,
                                   const Velocity &api, double api_age_s,
                                   const Limits &lim, double watchdog_s, double deadband = 0.05)
    {
        const auto off_centre = [deadband](double v) { return std::fabs(v) > deadband; };
        const bool stick_live = drone_mode and stick_age_s < watchdog_s and
                                (off_centre(stick.forward) or off_centre(stick.lateral) or
                                 off_centre(stick.climb) or off_centre(stick.yaw));
        if (stick_live)
            return {{stick.forward * lim.linear, stick.lateral * lim.linear,
                     stick.climb * lim.climb, stick.yaw * lim.yaw}, Source::Stick};

        if (api_age_s < watchdog_s)
        {
            // NaN would propagate into the integrated pose and teleport the node to nowhere.
            const auto clamp = [](double v, double m) { return std::isfinite(v) ? std::clamp(v, -m, m) : 0.0; };
            return {{clamp(api.forward, lim.linear), clamp(api.lateral, lim.linear),
                     clamp(api.climb, lim.climb), clamp(api.yaw, lim.yaw)}, Source::Program};
        }
        return {};
    }
}
