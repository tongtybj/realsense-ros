#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <iostream>
#include <mutex>


namespace realsense2_camera
{
  class T265Node
  {
  public:
    T265Node(ros::NodeHandle nh,
             ros::NodeHandle nhp,
             std::string frame,
             rs2::device dev,
             rs2::pipeline pipe):
      nh_(nh), nhp_(nhp),
      dev_(dev), pipe_(pipe), frame_(frame),
      base_ros_time_(-1), base_camera_time_(-1), frame_seq_(0)
    {
      /* ros param */
      nhp_.param("reset_duration", reset_duration_, 0.1);

      /* start device */
      start();
      ROS_INFO_STREAM("start t265 module: " << dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

      /* ros publisher */
      odom_pub_ = nh_.advertise<nav_msgs::Odometry>(frame_ + std::string("/odom"), 10);

      /* ros service */
      reset_srv_ = nh_.advertiseService(frame_ + "/reset", &T265Node::resetCb, this);
    }

    ~T265Node()
    {
      pipe_.stop();
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher odom_pub_;
    ros::ServiceServer reset_srv_;

    rs2::device dev_;
    rs2::pipeline pipe_;

    std::mutex mutex_;
    std::string frame_;
    double base_ros_time_;
    double base_camera_time_;
    double reset_duration_;
    int frame_seq_;

    void start()
    {
      rs2::config cfg;
      cfg.enable_device(dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
      cfg.disable_all_streams();
      cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

      /* we can only enable fhisheye both image */
      /*
        cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
        cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
      */

      rs2::pipeline_profile profiles = pipe_.start(cfg, std::bind(&T265Node::frameCb, this, std::placeholders::_1));
    }

    bool resetCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      pipe_.stop();

      //dev_.hardware_reset(); //error: hardware_reset is not implemented for this device!
      ros::Duration(reset_duration_).sleep();

      start();

      ROS_INFO_STREAM("reset t265 module: " << dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

      return true;
    }

    void frameCb(const rs2::frame& frame)
    {
      std::lock_guard<std::mutex> lock(mutex_);

      if(frame.is<rs2::pose_frame>())
        {
          double frame_time = frame.get_timestamp();

          if (base_ros_time_ < 0)
            {
              base_ros_time_ = ros::Time::now().toSec();
              base_camera_time_ = frame_time;
            }

          /*
            ROS_INFO("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
            rs2_stream_to_string(frame.get_profile().stream_type()),
            frame.get_profile().stream_index(),
            rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));
          */

          rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
          ros::Time t(base_ros_time_ + (frame_time - base_camera_time_) / 1000.0);


          if (odom_pub_.getNumSubscribers() > 0)
            {
              geometry_msgs::Pose pose_msg;
              pose_msg.position.x = -pose.translation.z;
              pose_msg.position.y = -pose.translation.x;
              pose_msg.position.z = pose.translation.y;
              pose_msg.orientation.x = -pose.rotation.z;
              pose_msg.orientation.y = -pose.rotation.x;
              pose_msg.orientation.z = pose.rotation.y;
              pose_msg.orientation.w = pose.rotation.w;

              geometry_msgs::Vector3 v_msg;
              v_msg.x = -pose.velocity.z;
              v_msg.y = -pose.velocity.x;
              v_msg.z = pose.velocity.y;

              geometry_msgs::Vector3 om_msg;
              om_msg.x = -pose.angular_velocity.z;
              om_msg.y = -pose.angular_velocity.x;
              om_msg.z = pose.angular_velocity.y;

              nav_msgs::Odometry odom_msg;
              frame_seq_ ++;

              odom_msg.header.frame_id = frame_ + std::string("_odom_frame");
              odom_msg.child_frame_id = frame_ + std::string("_pose_frame");
              odom_msg.header.stamp = t;
              odom_msg.header.seq = frame_seq_;
              odom_msg.pose.pose = pose_msg;
              odom_msg.twist.twist.linear = v_msg;
              odom_msg.twist.twist.angular = om_msg;
              odom_pub_.publish(odom_msg);
            }
        }
    }
  };

  class MultiT265Odom : public nodelet::Nodelet
  {
  public:
    MultiT265Odom()
    {
    }
    virtual ~MultiT265Odom()
    {
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    std::string frame_prefix_;

    rs2::context ctx_;
    std::vector<std::shared_ptr<T265Node> > camera_nodes_;

    void onInit() override
    {
      nh_ = getMTNodeHandle();
      nhp_ = getMTPrivateNodeHandle();

      nhp_.param("frame_prefix", frame_prefix_, std::string("realsense"));

      int index = 1;
      for (auto&& dev : ctx_.query_devices())
        {
          camera_nodes_.push_back(std::make_shared<T265Node>(nh_, nhp_, frame_prefix_ + std::to_string(index), dev, rs2::pipeline(ctx_)));
          index++;
        }

      std::cout << "finishi initialization" << std::endl;
    }
  };

};

PLUGINLIB_EXPORT_CLASS(realsense2_camera::MultiT265Odom, nodelet::Nodelet)
