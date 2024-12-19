#ifndef R4C_NAVIGATION_BT_TYPES_H
#define R4C_NAVIGATION_BT_TYPES_H

#include <behaviortree_cpp_v3/bt_factory.h>

// Utility functions for manually loading custom type variables in the BlackBoard.
// Please note that these should only be used for debugging and testing BT nodes.

// PoseStamped BT XML syntax: "frame_id;x;y;theta"
struct PoseStamped
{
    std::string frame_id;
    double x;
    double y;
    double theta;
};

template<>
PoseStamped BT::convertFromString(BT::StringView str)
{
    auto parts = BT::splitString(str, ';');
    PoseStamped output;
    output.frame_id = BT::convertFromString<std::string>(parts[0]);
    output.x        = BT::convertFromString<double>(parts[1]);
    output.y        = BT::convertFromString<double>(parts[2]);
    output.theta    = BT::convertFromString<double>(parts[3]);
    return output;
};

// ManeuverParameters BT XML syntax: 
// "global_frame;direction;type;flip_orientation;exit_dist;entry_dist;radius;waypoint_step"

struct ManeuverParameters
{
    std::string global_frame;
    int direction;
    int type;
    bool flip_orientation;
    double exit_dist;
    double entry_dist;
    double radius;
    double waypoint_step;
};

template<>
ManeuverParameters BT::convertFromString(BT::StringView str)
{
    auto parts = BT::splitString(str, ';');
    ManeuverParameters output;
    output.global_frame      = BT::convertFromString<std::string>(parts[0]);
    output.direction         = BT::convertFromString<int>(parts[1]);
    output.type              = BT::convertFromString<int>(parts[2]);
    output.flip_orientation  = BT::convertFromString<bool>(parts[3]);
    output.exit_dist         = BT::convertFromString<double>(parts[4]);
    output.entry_dist        = BT::convertFromString<double>(parts[5]);
    output.radius            = BT::convertFromString<double>(parts[6]);
    output.waypoint_step     = BT::convertFromString<double>(parts[7]);
    return output;
};

#endif  // R4C_NAVIGATION_BT_TYPES_H