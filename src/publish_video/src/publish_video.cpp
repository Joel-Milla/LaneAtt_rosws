//* Rclcpp needed code
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/utilities.hpp>

//* Messages libraries
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/image_encodings.hpp>  // Add this line!

//* Opencv libraries
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include "geometry_msgs/msg/accel.hpp"

class PublishImage : public rclcpp::Node {
private:
  static constexpr const char *NODE_NAME = "publish_image_node";
  static constexpr const char *PUB_IMG_TOPIC = "video_publisher";
  static constexpr const int STANDARD_QOS = 10;
  static constexpr const int FPS = 15;
  static constexpr const char *VIDEO_PATH =
      "/home/joel/Documents/research/RealTime-LaneATT/realsense/videos/entrenamiento_extra4.avi";

  using Image = sensor_msgs::msg::Image;

  rclcpp::Publisher<Image>::SharedPtr img_pub_;

  /*
  Need a function that loops infinitely, gets video avi, and publish it in a
  topic.

  Need:
  publisher<Image>
  */
  void publish_video(const std::string video_path) {
    cv::Mat frame;
    cv::VideoCapture cap;
    cv_bridge::CvImage cv_image;
    bool showWindow = false;
    rclcpp::Rate loop_rate(FPS);

    while (true && rclcpp::ok()) {
      cap.open(video_path);

      if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return;
      }

      while (true && rclcpp::ok()) {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
          std::cerr << "ERROR! blank frame grabbed\n";
          break;
        }

        // show live window
        if (showWindow)
          imshow("Live", frame);

        cv_image.header.stamp = this->get_clock()->now();
        cv_image.header.frame_id = "camera_frame";
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.image = frame;

        img_pub_->publish(*cv_image.toImageMsg());

        loop_rate.sleep();
      }
    }
  }

public:
  explicit PublishImage() : Node(NODE_NAME) {
    img_pub_ = this->create_publisher<Image>(PUB_IMG_TOPIC, STANDARD_QOS);
    publish_video(VIDEO_PATH);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PublishImage>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}