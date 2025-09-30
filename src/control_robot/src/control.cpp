//* Standard libraries
#include <cstddef>
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <utility>
#include <vector>

//* ROS2
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include <custom_interfaces/msg/prediction.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>

//* Boost libraries
#include <boost/circular_buffer.hpp>
#include <boost/math/statistics/linear_regression.hpp>

class BasicControl : public rclcpp::Node {

private:
  static constexpr const char *NODE_NAME = "control_basic_node";
  // static constexpr const char *COMMAND_VELOCITY_TOPIC = "/j100_0395/cmd_vel";
  static constexpr const char *COMMAND_VELOCITY_TOPIC = "cmd_vel";
  static constexpr const char *LANE_DETECTION_IMG_TOPIC =
      "lane_detection_output";
  static constexpr const char *PREDICTION_TOPIC_SUB = "prediction_video";
  static constexpr const int STANDARD_QOS = 10;

  static constexpr const size_t AMOUNT_BOTTOMMOST_POINTS = 7;
  static constexpr const int IMAGE_WIDTH = 640 - 1;
  static constexpr const int IMAGE_HEIGHT = 360 - 1;
  static constexpr const int IMAGE_CENTER = IMAGE_WIDTH / 2;
  static constexpr const float MAX_ANG_VEL = 1.0f;
  static constexpr const float MAX_LIN_VEL = 0.5f;
  static constexpr const float K_VALUE = 0.011111111f;
  static constexpr const int BUFFER_CAPACITY =
      30; //* Will hold only the 30 most relevant points

  static constexpr const int X = 0;
  static constexpr const int Y = 1;

  using Twist = geometry_msgs::msg::Twist;
  using Prediction = custom_interfaces::msg::Prediction;
  using Point = geometry_msgs::msg::Point;
  using Image = sensor_msgs::msg::Image;
  using Lane = std::vector<std::vector<int>>;

  rclcpp::Publisher<Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<Image>::SharedPtr pub_result_img;
  rclcpp::Subscription<Prediction>::SharedPtr sub_prediction_;

  boost::circular_buffer<float> left_lane_buffer_;
  boost::circular_buffer<float> right_lane_buffer_;

  /**
   * @brief Receives a vector of points, and returns the same values in a vector
   * of ints
   *
   * @param point_vector
   * @return Lane
   */
  std::pair<Lane, Lane>
  obtain_lanes(const std::vector<Point> &points_left_lane,
               const std::vector<Point> &points_right_lane) {
    Lane left_lane;
    left_lane.reserve(points_left_lane.size());
    for (const auto &point : points_left_lane) {
      //* If point.x is a negative value, it means that is out of bounds
      if (point.x >= 0)
        left_lane.push_back(
            {static_cast<int>(point.x), static_cast<int>(point.y)});
      else
        left_lane.push_back({0, static_cast<int>(point.y)});
    }

    Lane right_lane;
    right_lane.reserve(points_right_lane.size());
    for (const auto &point : points_right_lane) {
      //* If point.x is a negative value, it means that is out of bounds
      if (point.x >= 0)
        right_lane.push_back(
            {static_cast<int>(point.x), static_cast<int>(point.y)});
      else
        right_lane.push_back({IMAGE_WIDTH, static_cast<int>(point.y)});
    }

    //* Sort array based on y-value, from greatest to lowest
    auto descending_sort = [](const std::vector<int> &a,
                              const std::vector<int> &b) {
      return a[1] > b[1];
    };
    std::sort(left_lane.begin(), left_lane.end(), descending_sort);
    std::sort(right_lane.begin(), right_lane.end(), descending_sort);

    return {left_lane, right_lane};
  }

  /**
   * @brief Returns the middle row between two pairs of lanes
   *
   * @param left_lane
   * @param right_lane
   * @return Lane
   */
  Lane middle_row_between(const Lane &left_lane, const Lane &right_lane) {
    size_t indx_l = 0;
    size_t indx_r = 0;
    Lane middle_row;

    while (indx_l < left_lane.size() && indx_r < right_lane.size()) {
      //* If the y-coordinates match, get middle row
      if (left_lane[indx_l][Y] == right_lane[indx_r][Y]) {
        int middle_x = (left_lane[indx_l][X] + right_lane[indx_r][X]) / 2;
        middle_row.push_back({middle_x, left_lane[indx_l][Y]});
        indx_l++;
        indx_r++;
      }
      //* If the y-coordinates do not match, then continue until they do
      else {
        if (left_lane[indx_l][Y] > right_lane[indx_r][Y])
          indx_l++; //* Advance l because list is in descending order, and
                    // greater means need to advance to reduce
        else
          indx_r++;
      }
    }

    return middle_row;
  }

  void publish_and_visualize(const Image &img, const Lane lane1,
                             const Lane lane2, const Lane middle_lane,
                             int average, int angle_degrees, float dx, float dy,
                             float lin_vel, float ang_vel) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    int THICKNESS = 3;
    cv::polylines(cv_ptr->image, lane1, true, cv::Scalar(0, 255, 0), THICKNESS,
                  cv::LINE_8);
    cv::polylines(cv_ptr->image, lane2, true, cv::Scalar(0, 255, 0), THICKNESS,
                  cv::LINE_8);
    cv::polylines(cv_ptr->image, middle_lane, true, cv::Scalar(0, 0, 255),
                  THICKNESS, cv::LINE_8);

    //* Draw a line from top center of image, to the bottom of the image where
    // the lane_center was calculated. Show it in red
    size_t size = middle_lane.size();
    cv::line(cv_ptr->image, cv::Point(IMAGE_CENTER, IMAGE_HEIGHT),
             cv::Point(average, middle_lane[size - 1][Y]),
             cv::Scalar(255, 0, 0), THICKNESS);

    // Text box with debug info
    std::string text = "A:" + std::to_string(angle_degrees) +
                       " dx:" + std::to_string((int)dx) +
                       " dy:" + std::to_string((int)dy);
    int text_x = IMAGE_CENTER - 80;
    int text_y = 30;
    cv::Size text_size =
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
    cv::rectangle(cv_ptr->image,
                  cv::Point(text_x - 5, text_y - text_size.height - 5),
                  cv::Point(text_x + text_size.width + 5, text_y + 5),
                  cv::Scalar(255, 255, 255), -1);
    cv::putText(cv_ptr->image, text, cv::Point(text_x, text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

    // Second text box with velocity info
    std::string vel_text = "lin:" + std::to_string(lin_vel).substr(0, 4) +
                           " ang:" + std::to_string(ang_vel).substr(0, 4);
    int vel_y = text_y + 25;
    cv::Size vel_text_size =
        cv::getTextSize(vel_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
    cv::rectangle(cv_ptr->image,
                  cv::Point(text_x - 5, vel_y - vel_text_size.height - 5),
                  cv::Point(text_x + vel_text_size.width + 5, vel_y + 5),
                  cv::Scalar(255, 255, 255), -1);
    cv::putText(cv_ptr->image, vel_text, cv::Point(text_x, vel_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

    pub_result_img->publish(*cv_ptr->toImageMsg());
  }

  /**
   * @brief Obtain the prediction message, and perform the control defined
   * 
   * @param prediction_msg 
   */
  void prediction_subscriber(const Prediction::ConstSharedPtr &prediction_msg) {
    const std::vector<Point> &predicted_left_lane = prediction_msg->left_lane;
    const std::vector<Point> &predicted_right_lane = prediction_msg->right_lane;
    const Image &image = prediction_msg->frame;

    const auto [left_lane, right_lane] =
        obtain_lanes(predicted_left_lane, predicted_right_lane);

    Lane middle_lane = middle_row_between(left_lane, right_lane);
    if (middle_lane.size() == 0)
      return;

    middle_lane.resize(
        std::min(middle_lane.size(),
                 AMOUNT_BOTTOMMOST_POINTS)); //* Truncate the array to the first
                                             // AMOUNT_BOTTOMMOST_POINTS

    //* Obtain average of the first 7 points
    int average = 0;
    int num_points = middle_lane.size();
    for (const auto &point : middle_lane)
      average += point[X];
    average /= num_points;

    float dx = average - IMAGE_CENTER; // Horizontal difference
    float dy = IMAGE_HEIGHT -
               middle_lane[num_points - 1][Y]; // Vertical difference (negative
                                               // since y increases downward)

    float angle_radians = std::atan2(dy, dx);
    float angle_degrees = angle_radians * 180.0 / M_PI;
    float diff = angle_degrees - 90; //* Bring angle to [-90,90]

    //* Control calculation
    float lin_vel = (num_points / 7.0) * MAX_LIN_VEL;
    // float w = (diff * diff) / (90 * 90);
    float w = K_VALUE * diff;
    float ang_vel = MAX_ANG_VEL * w;

    if (diff < 0)
      ang_vel *= -1;

    lin_vel = lin_vel * (1 - w);
    publish_vel(lin_vel, ang_vel);
    publish_and_visualize(image, left_lane, right_lane, middle_lane, average,
                          diff, dx, dy, lin_vel, ang_vel);
  }

  /**
   * @brief Function solely to recieve linear and angular velocity, and publish ti
   * 
   * @param linear_x 
   * @param ang_z 
   */
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

    //* Initialize publisher and subscriber
    pub_cmd_vel_ =
        this->create_publisher<Twist>(COMMAND_VELOCITY_TOPIC, STANDARD_QOS);
    pub_result_img =
        this->create_publisher<Image>(LANE_DETECTION_IMG_TOPIC, STANDARD_QOS);
    sub_prediction_ = this->create_subscription<Prediction>(
        PREDICTION_TOPIC_SUB, STANDARD_QOS,
        [this](const Prediction::ConstSharedPtr &prediction) {
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