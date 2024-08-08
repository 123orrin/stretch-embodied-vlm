#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
//#include "geometry_msgs/msg/point.hpp"
//#include "geometry_msgs/msg/quaternion.hpp"

// Me
#include <spectacularAI/mapping.hpp>
#include <spectacularAI/realsense/plugin.hpp>

// SpectacularAI
#include <iostream>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <spectacularAI/realsense/plugin.hpp>
#include <spectacularAI/mapping.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <string>
#include <cstdint>
#include <thread>
#include <chrono>
#include <deque>
#include <atomic>
#include <cstdlib>
#include <set>
#include "helpers.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("sai_publisher"), count_(0)
  {
    publisher_rgbd_ = this->create_publisher<realsense2_camera_msgs::msg::RGBD>("/spectacular_ai/rgbd", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("/spectacular_ai/pose", 1);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message_rgbd = realsense2_camera_msgs::msg::RGBD();
    auto message_pose = geometry_msgs::msg::Pose();
    message_rgbd.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  
  geometry_msgs::msg::Pose* to_pose_msg(spectacularAI::Pose sai_pose)
  {
    auto pose = geometry_msgs::msg::Pose();
    pose->position->x = sai_pose->position->x
    pose->posiiton->y = sai_pose->position->y;
    pose->posiiton->z = sai_pose->position->z
    pose->orientation->x = sai_pose->quaternion->x;
    pose->orientation->y = sai_pose->quaternion->y;
    pose->orientation->z = sai_pose->quaternion->z;
    pose->orientation->w = sai_pose->quaternion->w;
    return &pose
  }
  

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<realsense2_camera_msgs::msg::RGBD>::SharedPtr publisher_rgbd_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_pose_;
  size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    //publisher_rgbd_ = this->create_publisher<realsense2_camera_msgs::msg::RGBD>("/spectacular_ai/rgbd", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("/spectacular_ai/pose", 1);
    
    /*
    spectacularAI::rsPlugin::Configuration vioConfig;
    spectacularAI::rsPlugin::Pipeline vioPipeline(vioConfig, [&](std::shared_ptr<const spectacularAI::mapping::MapperOutput> output){
        for (int64_t frameId : output->updatedKeyFrames) {
            auto search = output->map->keyFrames.find(frameId);
            if (search == output->map->keyFrames.end()) {
                continue; // deleted frame
            }

            auto& frameSet = search->second->frameSet;

            if (savedFrames.count(frameId) == 0) {
                // Only save images once, despide frames pose possibly being updated several times
                savedFrames.insert(frameId);
                std::lock_guard<std::mutex> lock(queueMutex);
                char *fileName = fileNameBuf.data();
                // Copy images to ensure they are in memory later for saving
                if (frameSet->rgbFrame && frameSet->rgbFrame->image) {
                    std::snprintf(fileName, fileNameBuf.size(), "%s/rgb_%04ld.png", recordingFolder.c_str(), frameId);
                    imageQueue.push_back(ImageToSave {fileName, copyImage(frameSet->rgbFrame->image)});
                }
                if (frameSet->depthFrame && frameSet->depthFrame->image) {
                    std::snprintf(fileName, fileNameBuf.size(), "%s/depth_%04ld.png", recordingFolder.c_str(), frameId);
                    imageQueue.push_back(ImageToSave {fileName, copyImage(frameSet->depthFrame->image)});
                }
                // TODO: Save pointclouds as JSON?
            }
        }
    });
    */    

    spectacularAI::rsPlugin::Pipeline vioPipeline(config);

    {
        // Find RealSense device
        rs2::context rsContext;
        rs2::device_list devices = rsContext.query_devices();
        if (devices.size() != 1) {
            std::cout << "More than one RealSense device connected. Using the first device detected." << std::endl;
        }
        rs2::device device = devices.front();
        vioPipeline.configureDevice(device);
    }

    rs2::config rsConfig;
    vioPipeline.configureStreams(rsConfig);
    auto vioSession = vioPipeline.startSession(rsConfig);
    

    while rclcpp:ok()
    {
        auto vioOut = vioSession->waitForOutput();
        RCLCPP_INFO(this->get_logger(), "GOT POSE");
        auto pose_msg = to_pose_msg(vioOut->Pose);
        publisher_pose_->publish(pose_msg);
    }

    rclcpp::shutdown();
    return 0;
}
