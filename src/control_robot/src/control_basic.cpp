//* Standard libraries
#include <cstdint>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <utility>
#include <vector>

//* ROS2
#include "cv_bridge/cv_bridge.h"
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
  static constexpr const char *PUB_RESULT_IMG = "control_result";
  static constexpr const char *SUB_PREDICTION_TOPIC = "prediction";
  static constexpr const int STANDARD_QOS = 10;

  static constexpr const int AMOUNT_BOTTOMMOST_POINTS = 10;
  static constexpr const int IMAGE_WIDTH = 1280;
  static constexpr const int IMAGE_HEIGHT = 720;
  static constexpr const int IMAGE_CENTER = IMAGE_WIDTH / 2;
  static constexpr const float MAX_ANG_VEL = 1.0f;
  static constexpr const float MAX_LIN_VEL = 1.0f;
  static constexpr const int BUFFER_CAPACITY =
      30; //* Will hold only the 30 most relevant points

  using Twist = geometry_msgs::msg::Twist;
  using Prediction = custom_interfaces::msg::Prediction;
  using Point = geometry_msgs::msg::Point;
  using Image = sensor_msgs::msg::Image;

  rclcpp::Publisher<Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<Image>::SharedPtr pub_result_img;
  rclcpp::Subscription<Prediction>::SharedPtr sub_prediction_;

  boost::circular_buffer<float> left_lane_buffer_;
  boost::circular_buffer<float> right_lane_buffer_;

  std::vector<int> bottom_most_points_;
  bool publish_results = false;
  bool show_results_ = false;

  /**
   * @brief Extracts from an array of Points (x,y,z) the x_points,y_points in
   * different arrays
   *
   * @param points
   * @return std::pair<std::vector<float> x_points, std::vector<float>> y_points
   */
  std::pair<std::vector<float>, std::vector<float>>
  separate_points(const std::vector<Point> &points) {
    std::vector<float> x_points;
    std::vector<float> y_points;

    //* Pre-allocate memory
    x_points.reserve(points.size());
    y_points.reserve(points.size());

    for (const Point &point : points) {
      x_points.push_back(point.x);
      y_points.push_back(point.y);
    }

    return {x_points, y_points};
  }

  void publish_and_visualize(const Image &img, const float lane_center_x) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    int lane_center = static_cast<int>(lane_center_x);
    //* Draw a line from top center of image, to the bottom of the image where
    // the lane_center was calculated. Show it in red
    cv::line(cv_ptr->image, cv::Point(lane_center, 0),
             cv::Point(lane_center, IMAGE_HEIGHT), cv::Scalar(0, 0, 255));

    //* Draw a line of the image center
    cv::line(cv_ptr->image, cv::Point(IMAGE_CENTER, 0),
             cv::Point(IMAGE_CENTER, IMAGE_HEIGHT - 1), cv::Scalar(0, 255, 0));

    if (show_results_) {
      cv::imshow("Lane center x Image center", cv_ptr->image);
      cv::waitKey(1);
    }

    pub_result_img->publish(*cv_ptr->toImageMsg());
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

    //* In each left/right buffer, save the x points of the y values at
    // bottom_most_points_
    for (const int &y : bottom_most_points_) {
      //* Get f(y) = c0 + c1 * y = x
      int left_x = static_cast<int>(left_c0 + (left_c1 * y));
      int right_x = static_cast<int>(right_c0 + (right_c1 * y));

      left_x = std::clamp(left_x, 0, IMAGE_WIDTH);
      right_x = std::clamp(right_x, 0, IMAGE_WIDTH);

      left_lane_buffer_.push_back(left_x);
      right_lane_buffer_.push_back(right_x);
    }

    //* Get the minimum safe of points to safely traverse both arrays
    int num_points =
        std::min(left_lane_buffer_.size(), right_lane_buffer_.size());
    float lane_center_x = 0.0;

    //* Get Xp, lane point in the center of the lane
    for (int indx = 0; indx < num_points; indx++)
      lane_center_x += (right_lane_buffer_[indx] + left_lane_buffer_[indx]) / 2;
    lane_center_x /= num_points;

    //* Angular/linear velocity
    float ang_z;
    float linear_x;

    //* Get total difference
    float difference = lane_center_x - IMAGE_CENTER;
    float alpha = (difference / IMAGE_CENTER) * (difference / IMAGE_CENTER);
    ang_z = MAX_ANG_VEL * alpha;
    ang_z = difference >= 0 ? -1 * ang_z : ang_z;

    if (publish_results)
      publish_and_visualize(prediction_msg->frame, lane_center_x);

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
    left_lane_buffer_ = boost::circular_buffer<float>(BUFFER_CAPACITY);
    right_lane_buffer_ = boost::circular_buffer<float>(BUFFER_CAPACITY);

    //* Initialize array that takes bottom most points
    bottom_most_points_.reserve(AMOUNT_BOTTOMMOST_POINTS);
    for (int val = IMAGE_HEIGHT - 1;
         val > (IMAGE_HEIGHT - AMOUNT_BOTTOMMOST_POINTS); val--)
      bottom_most_points_.push_back(val);

    //* Initialize publisher and subscriber
    pub_cmd_vel_ = this->create_publisher<Twist>(PUB_CMD_VEL, STANDARD_QOS);
    pub_result_img = this->create_publisher<Image>(PUB_RESULT_IMG, STANDARD_QOS);
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