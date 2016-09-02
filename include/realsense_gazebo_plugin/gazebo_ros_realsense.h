#ifndef _GAZEBO_ROS_REALSENSE_PLUGIN_
#define _GAZEBO_ROS_REALSENSE_PLUGIN_

#include "realsense_gazebo_plugin/RealSensePlugin.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <string>
#include <memory>

namespace gazebo
{
  /// \brief A plugin that simulates Real Sense camera streams.
  class GazeboRosRealsense : public RealSensePlugin
  {
    /// \brief Constructor.
    public: GazeboRosRealsense();

    /// \brief Destructor.
    public: ~GazeboRosRealsense();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that publishes a received Depth Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewDepthFrame();

    /// \brief Callback that publishes a received Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub);

    // /// \brief Put camera data to the ROS topic
    // protected:
    // void PutCameraData(const unsigned char *_src);
    // protected:
    // void PutCameraData(const unsigned char *_src,
    //                    common::Time &last_update_time);

    // /// \brief Publish CameraInfo to the ROS topic
    // protected:
    // void PublishCameraInfo(ros::Publisher camera_info_publisher);
    // protected:
    // void PublishCameraInfo(common::Time &last_update_time);
    // protected:
    // void PublishCameraInfo();

    protected: boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    protected: common::Time last_update_time_;

    /// \brief A pointer to the ROS node.
    ///  A node will be instantiated if it does not exist.
    protected: ros::NodeHandle* rosnode_;
    protected: image_transport::Publisher image_pub_;
    private: image_transport::ImageTransport* itnode_;

    /// \brief ROS image messages
    protected: sensor_msgs::Image image_msg_;
  };
}
#endif /* _GAZEBO_ROS_REALSENSE_PLUGIN_ */
