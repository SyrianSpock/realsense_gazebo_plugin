#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"
#include <sensor_msgs/fill_image.h>

namespace
{
  sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image);
}

namespace gazebo
{
// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

/////////////////////////////////////////////////
GazeboRosRealsense::GazeboRosRealsense()
{
}

/////////////////////////////////////////////////
GazeboRosRealsense::~GazeboRosRealsense()
{
  ROS_DEBUG_STREAM_NAMED("realsense_camera", "Unloaded");
}

/////////////////////////////////////////////////
void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  ROS_INFO("Realsense Gazebo ROS plugin loading.");

  RealSensePlugin::Load(_model, _sdf);

  this->rosnode_ = new ros::NodeHandle("/realsense");

  // initialize camera_info_manager
  this->camera_info_manager_.reset(
    new camera_info_manager::CameraInfoManager(*this->rosnode_, "realsense"));

  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  this->color_pub_ = this->itnode_->advertiseCamera("camera/color/image_raw", 2);
  this->ir1_pub_ = this->itnode_->advertiseCamera("camera/ir/image_raw", 2);
  this->ir2_pub_ = this->itnode_->advertiseCamera("camera/ir2/image_raw", 2);
  this->depth_pub_ = this->itnode_->advertiseCamera("camera/depth/image_raw", 2);
}

/////////////////////////////////////////////////
void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub)
{
  common::Time current_time = this->world->GetSimTime();

  // identify camera
  std::string camera_id = cam->Name();
  image_transport::CameraPublisher* image_pub;
  if (camera_id.find(COLOR_CAMERA_NAME) != std::string::npos)
  {
    camera_id = COLOR_CAMERA_NAME;
    image_pub = &(this->color_pub_);
  }
  else if (camera_id.find(IRED1_CAMERA_NAME) != std::string::npos)
  {
    camera_id = IRED1_CAMERA_NAME;
    image_pub = &(this->ir1_pub_);
  }
  else if (camera_id.find(IRED2_CAMERA_NAME) != std::string::npos)
  {
    camera_id = IRED2_CAMERA_NAME;
    image_pub = &(this->ir2_pub_);
  }
  else
  {
    ROS_ERROR("Unknown camera name\n");
    camera_id = COLOR_CAMERA_NAME;
    image_pub = &(this->color_pub_);
  }

  // copy data into image
  this->image_msg_.header.frame_id = camera_id;
  this->image_msg_.header.stamp.sec = current_time.sec;
  this->image_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = cam->ImageFormat();
  if (pixel_format == "L_INT8")
  {
    pixel_format = sensor_msgs::image_encodings::MONO8;
  }
  else if (pixel_format == "RGB_INT8")
  {
    pixel_format = sensor_msgs::image_encodings::RGB8;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat\n");
    pixel_format = sensor_msgs::image_encodings::BGR8;
  }

  // copy from simulation image to ROS msg
  fillImage(this->image_msg_,
    pixel_format,
    cam->ImageHeight(), cam->ImageWidth(),
    cam->ImageDepth() * cam->ImageWidth(),
    reinterpret_cast<const void*>(cam->ImageData()));

  sensor_msgs::CameraInfo cam_info_msg = cameraInfo(this->image_msg_);

  // publish to ROS
  image_pub->publish(this->image_msg_, cam_info_msg);
}

/////////////////////////////////////////////////
void GazeboRosRealsense::OnNewDepthFrame()
{
  // get current time
  common::Time current_time = this->world->GetSimTime();

  RealSensePlugin::OnNewDepthFrame();

  // copy data into image
  this->depth_msg_.header.frame_id = COLOR_CAMERA_NAME;
  this->depth_msg_.header.stamp.sec = current_time.sec;
  this->depth_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

  // copy from simulation image to ROS msg
  fillImage(this->depth_msg_,
    pixel_format,
    this->depthCam->ImageHeight(), this->depthCam->ImageWidth(),
    2 * this->depthCam->ImageWidth(),
    reinterpret_cast<const void*>(this->depthMap.data()));

  sensor_msgs::CameraInfo depth_info_msg = cameraInfo(this->depth_msg_);

  // publish to ROS
  this->depth_pub_.publish(this->depth_msg_, depth_info_msg);
}

}

namespace
{
  sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image)
  {
    sensor_msgs::CameraInfo info_msg;

    info_msg.header = image.header;
    info_msg.height = image.height;
    info_msg.width = image.width;

    float focal = 463.889;

    info_msg.K[0] = focal;
    info_msg.K[4] = focal;
    info_msg.K[2] = info_msg.width * 0.5;
    info_msg.K[5] = info_msg.height * 0.5;
    info_msg.K[8] = 1.;

    info_msg.P[0] = info_msg.K[0];
    info_msg.P[5] = info_msg.K[4];
    info_msg.P[2] = info_msg.K[2];
    info_msg.P[6] = info_msg.K[5];
    info_msg.P[10] = info_msg.K[8];

    return info_msg;
  }
}

