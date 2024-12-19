#ifndef REPORTER_COMMONS_H_
#define REPORTER_COMMONS_H_

#pragma once

#include <cstdint>
#include <set>
#include "rclcpp/rclcpp.hpp"
#include <r4c_interfaces/msg/can_status.hpp>
#include <r4c_interfaces/msg/communication_status.hpp>
#include <r4c_interfaces/msg/internet_status.hpp>
#include <r4c_interfaces/msg/localization_status.hpp>
#include <r4c_interfaces/msg/sensor_status.hpp>
#include <r4c_interfaces/msg/sensors_status.hpp>
#include <r4c_interfaces/msg/tractor_status.hpp>
#include "r4c_reporter/status_transitions.hpp"

namespace reporter_commons
{
  template <class MsgT, class StatusT>
  void evaluateConnection(StatusT& sensor_status, const typename MsgT::SharedPtr& sensor_msg, rclcpp::Time current_time, float timeout)
  {
    rclcpp::Duration time_diff = current_time - sensor_msg->header.stamp;

    double time_difference = time_diff.seconds();

    if(time_difference > timeout)
    {
      state_transitions::addErrorCode<StatusT>(StatusT::DISCONNECTED, sensor_status);
    }
    else
    {
      state_transitions::removeErrorCode<StatusT>(StatusT::DISCONNECTED, sensor_status);
    }
  }

  template <class MsgT, class StatusT>
  void evaluateSynchronization(StatusT& sensor_status, const typename MsgT::SharedPtr& sensor_msg, rclcpp::Time current_time, float sync_threshold)
  {
    rclcpp::Duration time_diff = current_time - sensor_msg->header.stamp;

    double time_difference = time_diff.seconds();

    if(time_difference > sync_threshold)
    {
      state_transitions::addErrorCode<StatusT>(StatusT::NOT_SYNCED, sensor_status);
    }
    else
    {
      state_transitions::removeErrorCode<StatusT>(StatusT::NOT_SYNCED, sensor_status);
    }
  }

}  // namespace reporter_commons

#endif