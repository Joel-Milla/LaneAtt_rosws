//* Standard libraries
#include <cstdint>
#include <utility>
#include <vector>

//* ROS2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include <custom_interfaces/msg/detail/prediction__struct.hpp>
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

  static constexpr const int AMOUNT_BOTTOMMOST_POINTS = 10;
  static constexpr const int IMAGE_WIDTH = 1280;
  static constexpr const int IMAGE_HEIGHT = 720;
  static constexpr const int IMAGE_CENTER = IMAGE_WIDTH / 2;
  static constexpr const float MAX_ANG_VEL = 1.0f;
  static constexpr const float MAX_LIN_VEL = 1.0f;

  using Twist = geometry_msgs::msg::Twist;
  using Prediction = custom_interfaces::msg::Prediction;
  using Point = geometry_msgs::msg::Point;

  rclcpp::Publisher<Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<Prediction>::SharedPtr sub_prediction_;

  boost::circular_buffer<float> left_lane_buffer_;
  boost::circular_buffer<float> right_lane_buffer_;

  std::vector<int> bottom_most_points_;

  /**
   * @brief Extracts from an array of Points (x,y,z) the x_points,y_points in
   * different arrays
   *
   * @param points
   * @return std::pair<std::vector<float> x_points, std::vector<float>> y_points
   */
  std::pair<std::vector<float>, std::vector<float>>
  separate_points(const std::vector<Point> &points) {
    std::vector<float> x_points(points.size());
    std::vector<float> y_points(points.size());

    for (const Point &point : points) {
      x_points.push_back(point.x);
      y_points.push_back(point.y);
    }

    return {x_points, y_points};
  }

  void prediction_subscriber(const Prediction::ConstSharedPtr &prediction_msg) {
    const std::vector<Point> &predicted_left_lane = prediction_msg->left_lane;
    const std::vector<Point> &predicted_right_lane = prediction_msg->right_lane;

    //* Saved the points predicted into the buffer
    const auto [left_x_points, left_y_points] =
        separate_points(predicted_left_lane);

    const auto [right_x_points, right_y_points] =
        separate_points(predicted_right_lane);

    using boost::math::statistics::simple_ordinary_least_squares;
    auto [left_c0, left_c1] =
        simple_ordinary_least_squares(left_y_points, left_x_points);
    auto [right_c0, right_c1] =
        simple_ordinary_least_squares(right_y_points, right_x_points);

    //* In each left/right buffer, save the x points of the y values at bottom_most_points_
    for (const int &y : bottom_most_points_) {
      //* Get f(y) = c0 + c1 * y = x
      int left_x = static_cast<int>(left_c0 + (left_c1 * y));
      int right_x = static_cast<int>(right_c0 + (right_c1 * y));

      left_x = std::clamp(left_x, 0, IMAGE_WIDTH);
      right_x = std::clamp(right_x, 0, IMAGE_WIDTH);

      left_lane_buffer_.push_back(left_x);
      right_lane_buffer_.push_back(right_x);
    }

    int num_points = std::min(left_lane_buffer_.size(), right_lane_buffer_.size());
    int average = 0;

    //* Get Xp, lane point in the center of the lane
    for (int indx = 0; indx < num_points; indx++)
      average += (right_lane_buffer_[indx] - left_lane_buffer_[indx]) / 2;
    average /= num_points;

    //* Angular/linear velocity
    float ang_z;
    float linear_x;

    //* Get total difference
    float difference = average - IMAGE_CENTER;
    float alpha = (difference / IMAGE_CENTER) * (difference / IMAGE_CENTER);
    ang_z = MAX_ANG_VEL * alpha;
    ang_z = -1 * ang_z ? difference >= 0 : ang_z;

    linear_x = MAX_LIN_VEL * (1 - alpha);
    publish_vel(linear_x, ang_z);
  }

  void publish_vel(float linear_x, float ang_z) {
    Twist msg;
    msg.linear.set__x(linear_x);
    msg.angular.set__z(ang_z);

    pub_cmd_vel_->publish(msg);
  }

public:
  explicit BasicControl() : Node(NODE_NAME) {
    left_lane_buffer_ = boost::circular_buffer<float>(30);
    right_lane_buffer_ = boost::circular_buffer<float>(30);

    //* Initialize array that takes bottom most points
    bottom_most_points_.reserve(AMOUNT_BOTTOMMOST_POINTS);
    for (int val = IMAGE_HEIGHT - 1;
         val > (IMAGE_HEIGHT - AMOUNT_BOTTOMMOST_POINTS); val--)
      bottom_most_points_.push_back(val);

    pub_cmd_vel_ = this->create_publisher<Twist>(PUB_CMD_VEL, STANDARD_QOS);
    sub_prediction_ = this->create_subscription<Prediction>(
        SUB_PREDICTION_TOPIC, STANDARD_QOS,
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