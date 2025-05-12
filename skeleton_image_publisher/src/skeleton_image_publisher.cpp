#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <openni2/OpenNI.h>
#include <NiTE.h>

class SkeletonImagePublisher : public rclcpp::Node {
public:
  SkeletonImagePublisher() : Node("skeleton_image_publisher") {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("skeleton_image", 10);

    if (openni::OpenNI::initialize() != openni::STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "OpenNI initialization failed: %s", openni::OpenNI::getExtendedError());
      rclcpp::shutdown();
      return;
    }

    if (device_.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "Device open failed: %s", openni::OpenNI::getExtendedError());
      rclcpp::shutdown();
      return;
    }

    if (color_stream_.create(device_, openni::SENSOR_COLOR) != openni::STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "Color stream creation failed: %s", openni::OpenNI::getExtendedError());
      rclcpp::shutdown();
      return;
    }
    if (depth_stream_.create(device_, openni::SENSOR_DEPTH) != openni::STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "Depth stream creation failed: %s", openni::OpenNI::getExtendedError());
      rclcpp::shutdown();
      return;
    }
    color_stream_.start();
    depth_stream_.start();

    if (nite::NiTE::initialize() != nite::STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "NiTE initialization failed!");
      rclcpp::shutdown();
      return;
    }

    if (user_tracker_.create(&device_) != nite::STATUS_OK) {
      RCLCPP_ERROR(get_logger(), "User tracker creation failed!");
      rclcpp::shutdown();
      return;
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(33), std::bind(&SkeletonImagePublisher::publish_image, this));
    RCLCPP_INFO(get_logger(), "SkeletonImagePublisher initialized successfully!");
  }

  ~SkeletonImagePublisher() {
    user_tracker_.destroy();
    color_stream_.stop();
    depth_stream_.stop();
    device_.close();
    nite::NiTE::shutdown();
    openni::OpenNI::shutdown();
  }

private:
  void publish_image() {
    nite::UserTrackerFrameRef user_frame;
    if (user_tracker_.readFrame(&user_frame) != nite::STATUS_OK) {
      RCLCPP_WARN(get_logger(), "Failed to read user frame!");
      return;
    }

    // Depth görüntüsünü al
    openni::VideoFrameRef depth_frame;
    if (depth_stream_.readFrame(&depth_frame) != openni::STATUS_OK) {
      RCLCPP_WARN(get_logger(), "Failed to read depth frame!");
      return;
    }
    cv::Mat depth_image(depth_frame.getHeight(), depth_frame.getWidth(), CV_16U, (void*)depth_frame.getData());
    cv::Mat depth_display;
    depth_image.convertTo(depth_display, CV_8U, 255.0 / 4000.0); // 4000 mm maksimum mesafe varsayımı
    cv::cvtColor(depth_display, depth_display, cv::COLOR_GRAY2BGR);

    // Kullanıcı segmentasyonunu al
    const nite::UserMap& user_map = user_frame.getUserMap();
    const nite::UserId* user_labels = user_map.getPixels();
    int width = user_map.getWidth();
    int height = user_map.getHeight();

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        nite::UserId label = user_labels[y * width + x];
        if (label != 0) {
          depth_display.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Kırmızı
        }
      }
    }

    // Kullanıcı ve skeleton verilerini işle
    const nite::Array<nite::UserData>& users = user_frame.getUsers();
    RCLCPP_INFO(get_logger(), "Found %d users", users.getSize());
    for (int i = 0; i < users.getSize(); ++i) {
      const nite::UserData& user = users[i];
      RCLCPP_INFO(get_logger(), "User %d isVisible: %d, isLost: %d, isNew: %d", user.getId(), user.isVisible(), user.isLost(), user.isNew());

      if (!user.isVisible()) {
        RCLCPP_INFO(get_logger(), "User %d is not visible", user.getId());
        continue;
      }

      if (user.isLost()) {
        RCLCPP_INFO(get_logger(), "User %d is lost", user.getId());
        continue;
      }

      if (user.isNew()) {
        RCLCPP_INFO(get_logger(), "User %d is new, starting skeleton tracking...", user.getId());
        user_tracker_.startSkeletonTracking(user.getId());
      }

      const nite::Skeleton& skeleton = user.getSkeleton();
      if (skeleton.getState() == nite::SKELETON_CALIBRATING) {
        RCLCPP_INFO(get_logger(), "User %d is calibrating...", user.getId());
        continue;
      }
      if (skeleton.getState() != nite::SKELETON_TRACKED) {
        RCLCPP_INFO(get_logger(), "User %d skeleton state: %d (not tracked)", user.getId(), skeleton.getState());
        continue;
      }

      RCLCPP_INFO(get_logger(), "User %d skeleton tracked!", user.getId());
      // Eklemleri çiz
      for (int j = 0; j < 15; ++j) {
        nite::SkeletonJoint joint = skeleton.getJoint(static_cast<nite::JointType>(j));
        if (joint.getPositionConfidence() > 0.3) {
          nite::Point3f pos = joint.getPosition();
          float x, y;
          user_tracker_.convertJointCoordinatesToDepth(pos.x, pos.y, pos.z, &x, &y);
          RCLCPP_INFO(get_logger(), "Joint %d: x=%.2f, y=%.2f, confidence=%.2f", j, x, y, joint.getPositionConfidence());
          cv::circle(depth_display, cv::Point(static_cast<int>(x), static_cast<int>(y)), 5, cv::Scalar(0, 255, 0), -1);
        } else {
          RCLCPP_INFO(get_logger(), "Joint %d confidence too low: %.2f", j, joint.getPositionConfidence());
        }
      }

      // Eklemler arası çizgileri çiz
      struct JointConnection {
        nite::JointType joint1;
        nite::JointType joint2;
      };
      JointConnection connections[] = {
        {nite::JOINT_HEAD, nite::JOINT_NECK},
        {nite::JOINT_NECK, nite::JOINT_LEFT_SHOULDER},
        {nite::JOINT_LEFT_SHOULDER, nite::JOINT_LEFT_ELBOW},
        {nite::JOINT_LEFT_ELBOW, nite::JOINT_LEFT_HAND},
        {nite::JOINT_NECK, nite::JOINT_RIGHT_SHOULDER},
        {nite::JOINT_RIGHT_SHOULDER, nite::JOINT_RIGHT_ELBOW},
        {nite::JOINT_RIGHT_ELBOW, nite::JOINT_RIGHT_HAND},
        {nite::JOINT_LEFT_SHOULDER, nite::JOINT_TORSO},
        {nite::JOINT_RIGHT_SHOULDER, nite::JOINT_TORSO},
        {nite::JOINT_TORSO, nite::JOINT_LEFT_HIP},
        {nite::JOINT_LEFT_HIP, nite::JOINT_LEFT_KNEE},
        {nite::JOINT_LEFT_KNEE, nite::JOINT_LEFT_FOOT},
        {nite::JOINT_TORSO, nite::JOINT_RIGHT_HIP},
        {nite::JOINT_RIGHT_HIP, nite::JOINT_RIGHT_KNEE},
        {nite::JOINT_RIGHT_KNEE, nite::JOINT_RIGHT_FOOT}
      };

      for (const auto& conn : connections) {
        nite::SkeletonJoint joint1 = skeleton.getJoint(conn.joint1);
        nite::SkeletonJoint joint2 = skeleton.getJoint(conn.joint2);
        if (joint1.getPositionConfidence() > 0.3 && joint2.getPositionConfidence() > 0.3) {
          nite::Point3f pos1 = joint1.getPosition();
          nite::Point3f pos2 = joint2.getPosition();
          float x1, y1, x2, y2;
          user_tracker_.convertJointCoordinatesToDepth(pos1.x, pos1.y, pos1.z, &x1, &y1);
          user_tracker_.convertJointCoordinatesToDepth(pos2.x, pos2.y, pos2.z, &x2, &y2);
          cv::line(depth_display, cv::Point(static_cast<int>(x1), static_cast<int>(y1)), cv::Point(static_cast<int>(x2), static_cast<int>(y2)), cv::Scalar(0, 255, 0), 2);
        }
      }
    }

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", depth_display).toImageMsg();
    msg->header.frame_id = "kinect_frame";
    msg->header.stamp = now();
    image_pub_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  openni::Device device_;
  openni::VideoStream color_stream_, depth_stream_;
  nite::UserTracker user_tracker_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SkeletonImagePublisher>());
  rclcpp::shutdown();
  return 0;
}