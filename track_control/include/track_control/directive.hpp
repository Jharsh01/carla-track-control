// What a supervisor, such as a vision-language-action (VLA) model, may ask of the NMPC while it drives.
//
// A `Directive` overrides some of the NMPC's settings for a while:
//
//   weights  the cost weights, by term name (NMPC::weight_ranges() lists them): speed, lane, lat, accel,
//            rate, slack
//   limits   by name (NMPC::limit_ranges()): speed_cap [m/s], lane_offset [m], margin_lat, margin_long,
//            edge_margin [m], sideslip_max [rad]
//   zones    places to keep out of, in path coordinates: a pedestrian, debris, a closed lane
//
// Whatever is not named keeps its default. The NMPC clamps every value to its range, and the ranges only let
// a directive make the car more careful than the defaults, or tune its performance within limits: the speed
// cap cannot exceed the road's limit, the margins and the slack weight cannot drop below their defaults, the
// sideslip limit cannot rise above its default, and zones only ever add constraints. The road edges, the
// tire friction and the actuator limits are not the directive's to change.
//
// A directive lapses `ttl` seconds after it took effect and the defaults return, so a supervisor that stops
// talking leaves the car on its defaults. A VLA that speaks in driving intents goes through NMPC::interpret.
#pragma once

#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace tc {

// A place to keep out of: a rectangle along the path, `length` x `width` [m], centred at (s, e_y) at the
// directive's time, moving along the path at `speed` [m/s]. The NMPC keeps the ego body clear of it by the
// margins, as a soft constraint like the one for cars.
struct Zone {
    double s = 0.0;  // [m]
    double e_y = 0.0;  // [m], to the left
    double length = 1.0;  // [m]
    double width = 1.0;  // [m]
    double speed = 0.0;  // [m/s]
};

struct Directive {
    double t = NAN;  // when it was issued [s], on the observations' clock; NaN: at the next solve
    double ttl = 2.0;  // it lapses this long after t [s]
    std::map<std::string, double> weights;
    std::map<std::string, double> limits;
    std::vector<Zone> zones;
};

// A driving intent, as a VLA model might name it.
struct Intent {
    enum Kind {
        keep_lane,  // back to the home lane
        lane_left,  // the lane left of the home lane
        lane_right,  // the lane right of the home lane
        set_speed,  // cap the speed at `value` [m/s]
        more_gap,  // keep `value` (default 2) times the default margins to other cars and zones
        cautious,  // smoother inputs (`value`, default 3, times the input weights) and half the sideslip
        avoid,  // keep out of `zone`
    };
    Intent(Kind kind = keep_lane, double value = NAN, Zone zone = Zone()) : kind(kind), value(value), zone(zone) {}
    Kind kind;
    double value;
    Zone zone;
};

// The intent named `name` ("keep_lane", "lane_left", ...), for a model that answers in text; throws
// std::invalid_argument for an unknown name.
Intent::Kind intent_kind(const std::string& name);

}  // namespace tc
