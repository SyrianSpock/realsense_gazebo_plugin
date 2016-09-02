#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"
#include <sensor_msgs/fill_image.h>

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

  this->image_pub_ = this->itnode_->advertise("realsense/camera", 2);
}

/////////////////////////////////////////////////
void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub)
{
  this->last_update_time_ = this->world->GetSimTime();

  // identify camera
  std::string camera_id = cam->Name();
  if (camera_id.find(COLOR_CAMERA_NAME) != std::string::npos)
  {
    camera_id = COLOR_CAMERA_NAME;
  }
  else if (camera_id.find(IRED1_CAMERA_NAME) != std::string::npos)
  {
    camera_id = IRED1_CAMERA_NAME;
  }
  else if (camera_id.find(IRED2_CAMERA_NAME) != std::string::npos)
  {
    camera_id = IRED2_CAMERA_NAME;
  }
  else
  {
    ROS_ERROR("Unknown camera name\n");
    camera_id = COLOR_CAMERA_NAME;
  }

  // copy data into image
  this->image_msg_.header.frame_id = camera_id;
  this->image_msg_.header.stamp.sec = this->last_update_time_.sec;
  this->image_msg_.header.stamp.nsec = this->last_update_time_.nsec;

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
  ROS_INFO_STREAM("Camera" << cam->Name() << "Image format " << cam->ImageFormat());

  // copy from simulation image to ROS msg
  fillImage(this->image_msg_,
    pixel_format,
    cam->ImageHeight(), cam->ImageWidth(),
    cam->ImageDepth() * cam->ImageWidth(),
    reinterpret_cast<const void*>(cam->ImageData()));

  // publish to ROS
  this->image_pub_.publish(this->image_msg_);
}

/////////////////////////////////////////////////
void GazeboRosRealsense::OnNewDepthFrame()
{
}

////////////////////////////////////////////////////////////////////////////////
// void GazeboRosRealsense::PutCameraData(const unsigned char *_src,
//   common::Time &last_update_time)
// {
//   this->sensor_update_time_ = last_update_time;

//   if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
//     return;

//   /// don't bother if there are no subscribers
//   if ((*this->image_connect_count_) > 0)
//   {
//     boost::mutex::scoped_lock lock(this->lock_);

//     // copy data into image
//     this->image_msg_.header.frame_id = this->frame_name_;
//     this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
//     this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

//     // copy from src to image_msg_
//     fillImage(this->image_msg_, this->type_, this->height_, this->width_,
//         this->skip_*this->width_, reinterpret_cast<const void*>(_src));

//     // publish to ros
//     this->image_pub_.publish(this->image_msg_);
//   }
// }

////////////////////////////////////////////////////////////////////////////////
// void GazeboRosRealsense::PublishCameraInfo(common::Time &last_update_time)
// {
//   if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
//     return;

//   this->sensor_update_time_ = last_update_time;
//   this->PublishCameraInfo();
// }

// void GazeboRosRealsense::PublishCameraInfo()
// {
//   if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
//     return;

//   if (this->camera_info_pub_.getNumSubscribers() > 0)
//   {
// # if GAZEBO_MAJOR_VERSION >= 7
//     this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
// # else
//     this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
// # endif
//     common::Time cur_time = this->world_->GetSimTime();
//     if (cur_time - this->last_info_update_time_ >= this->update_period_)
//     {
//       this->PublishCameraInfo(this->camera_info_pub_);
//       this->last_info_update_time_ = cur_time;
//     }
//   }
// }

// void GazeboRosRealsense::PublishCameraInfo(
//   ros::Publisher camera_info_publisher)
// {
//   sensor_msgs::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();

//   camera_info_msg.header.stamp.sec = this->sensor_update_time_.sec;
//   camera_info_msg.header.stamp.nsec = this->sensor_update_time_.nsec;

//   camera_info_publisher.publish(camera_info_msg);
// }

}
