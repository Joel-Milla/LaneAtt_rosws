//* Standard libraries
#include <vector>

//* ROS2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <custom_interfaces/msg/detail/prediction__struct.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/detail/point__struct.hpp>

//* Boost libraries
#include <boost/circular_buffer.hpp>
#include <boost/math/statistics/linear_regression.hpp>

class BasicControl : public rclcpp::Node {

private:
  static constexpr const char *NODE_NAME = "control_basic_node";
  static constexpr const char *PUB_CMD_VEL = "cmd_vel";
  static constexpr const char *SUB_PREDICTION_TOPIC = "prediction";
  static constexpr const int STANDARD_QOS = 10;

  using Twist = geometry_msgs::msg::Twist;
  using Prediction = custom_interfaces::msg::Prediction;
  using Point = geometry_msgs::msg::Point;

  rclcpp::Publisher<Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<Prediction>::SharedPtr sub_prediction_;

  boost::circular_buffer<float> left_lane_;
  boost::circular_buffer<float> right_lane_;

  /**
   * @brief Saves predicted values into the ring buffers
   * 
   * @param points to be saved
   * @param buffer data structure to save the points
   */
  void append_values(const std::vector<Point> &points, boost::circular_buffer<float> &buffer) {
    for (const Point &point : points)
      buffer.push_back(point.x);
  }

  void prediction_subscriber(const Prediction::ConstSharedPtr &prediction_msg) {
    const std::vector<Point> &predicted_left_lane = prediction_msg->left_lane;
    const std::vector<Point> &predicted_right_lane = prediction_msg->right_lane;

    //* Saved the points predicted into the buffer
    append_values(predicted_left_lane, left_lane_);
    append_values(predicted_right_lane, right_lane_);


  }

  void publish_vel(float linear_x, float ang_z) {
    Twist msg;
    msg.linear.set__x(linear_x);
    msg.angular.set__z(ang_z);

    pub_cmd_vel_->publish(msg);
  }

public:
  explicit BasicControl() : Node(NODE_NAME) {
    left_lane_ = boost::circular_buffer<float>(30); // max size 5
    right_lane_ = boost::circular_buffer<float>(30); // max size 5
    
    pub_cmd_vel_ = this->create_publisher<Twist>(PUB_CMD_VEL, STANDARD_QOS);
    sub_prediction_ = this->create_subscription<Prediction>(SUB_PREDICTION_TOPIC, STANDARD_QOS, 
      [this](Prediction::ConstSharedPtr prediction) {
        this->prediction_subscriber(prediction);
      });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BasicControl>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}