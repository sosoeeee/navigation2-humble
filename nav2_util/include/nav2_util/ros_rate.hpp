#ifndef NAV2_UTIL__ROS_RATE_HPP_
#define NAV2_UTIL__ROS_RATE_HPP_

// #include <vector>
// #include <string>
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

class RosRate
{
public:
  RosRate(double rate, rclcpp::Clock::SharedPtr clock)
  : RosRate(rclcpp::Duration::from_seconds(1.0 / rate), clock)
  {
    if (rate <= 0.0) {
    throw std::invalid_argument{"rate must be greater than 0"};
    }
    period_ = Duration::from_seconds(1.0 / rate);
  }

  RosRate(rclcpp::Duration period, rclcpp::Clock::SharedPtr clock)
  : period_(period), last_interval_(clock->now()), clock_(clock)
  {
    if (period <= Duration(0, 0)) {
    throw std::invalid_argument{"period must be greater than 0"};
    }
  }

  bool sleep()
  {
    // Time coming into sleep
    auto now = clock_->now();
    // Time of next interval
    auto next_interval = last_interval_ + period_;
    // Detect backwards time flow
    if (now < last_interval_) {
        // Best thing to do is to set the next_interval to now + period
        next_interval = now + period_;
    }
    // Update the interval
    last_interval_ += period_;
    // If the time_to_sleep is negative or zero, don't sleep
    if (next_interval <= now) {
        // If an entire cycle was missed then reset next interval.
        // This might happen if the loop took more than a cycle.
        // Or if time jumps forward.
        if (now > next_interval + period_) {
        last_interval_ = now + period_;
        }
        // Either way do not sleep and return false
        return false;
    }
    // Calculate the time to sleep
    auto time_to_sleep = next_interval - now;
    // Sleep (will get interrupted by ctrl-c, may not sleep full time)
    return clock_->sleep_for(time_to_sleep);
  }

  void reset()
  {
    last_interval_ = clock_->now();
  }

  rclcpp::Duration period() const
  {
    return period_;
  }

protected:
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__ROS_RATE_HPP_
