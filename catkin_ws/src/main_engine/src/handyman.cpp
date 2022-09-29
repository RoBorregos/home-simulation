#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <handyman/HandymanMsg.h>
#include <sensor_msgs/JointState.h>
#include <nodelet/nodelet.h>
#include <actionlib/client/simple_action_client.h>
#include <object_detector/DetectObjects3DAction.h>

using namespace object_detector;
typedef actionlib::SimpleActionClient<DetectObjects3DAction> Detect3DClient;

class HandymanMain
{
private:
  enum Step
  {
    Initialize,
    Ready,
    WaitForInstruction,
    GoToRoom1,
    GoToRoom2,
    MoveToInFrontOfTarget,
    Grasp,
    WaitForGrasping,
    ComeBack,
    TaskFinished,
  };

  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_INSTRUCTION      = "Instruction";
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";

  const std::string MSG_I_AM_READY     = "I_am_ready";
  const std::string MSG_ROOM_REACHED   = "Room_reached";
  const std::string MSG_OBJECT_GRASPED = "Object_grasped";
  const std::string MSG_TASK_FINISHED  = "Task_finished";

  trajectory_msgs::JointTrajectory arm_joint_trajectory_;

  int step_;

  std::string instruction_msg_;

  bool is_started_;
  bool is_finished_;
  bool is_failed_;

  ros::Publisher  pub_arm_trajectory;
  ros::Publisher  pub_gripper_trajectory;
  ros::Publisher  pub_head_trajectory;

  // clear_octomap
  ros::ServiceClient clear_octomap;


  ros::NodeHandle nh_;
  Detect3DClient ac_detection3D; 

  void init()
  {
    // Arm Joint Trajectory
    std::vector<std::string> arm_joint_names {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

    trajectory_msgs::JointTrajectoryPoint arm_joint_point;

    arm_joint_trajectory_.joint_names = arm_joint_names;
    arm_joint_trajectory_.points.push_back(arm_joint_point);

    step_ = Initialize;

    reset();
  }

  void reset()
  {
    instruction_msg_ = "";
    is_started_  = false;
    is_finished_ = false;
    is_failed_   = false;

    std::vector<double> arm_positions { 0.0, 0.0, 1.5708, -1.5708, 0.0 };
    arm_joint_trajectory_.points[0].positions = arm_positions;
    arm_joint_trajectory_.points[0].time_from_start = ros::Duration(1.75);
  }


  void messageCallback(const handyman::HandymanMsg::ConstPtr& message)
  {
    ROS_INFO("Subscribe message:%s, %s", message->message.c_str(), message->detail.c_str());

    if(message->message.c_str()==MSG_ARE_YOU_READY)
    {
      if(step_==Ready)
      {
        is_started_ = true;
      }
    }
    if(message->message.c_str()==MSG_INSTRUCTION)
    {
      if(step_==WaitForInstruction)
      {
        instruction_msg_ = message->detail.c_str();
      }
    }
    if(message->message.c_str()==MSG_TASK_SUCCEEDED)
    {
      if(step_==TaskFinished)
      {
        is_finished_ = true;
      }
    }
    if(message->message.c_str()==MSG_TASK_FAILED)
    {
      is_failed_ = true;
    }
    if(message->message.c_str()==MSG_MISSION_COMPLETE)
    {
      exit(EXIT_SUCCESS);
    }
  }

  void trajectoriesCallback(const sensor_msgs::JointState::ConstPtr& message)
  {
    const double fake_controller_rate = 50.0; 
    ros::Duration duration(1/fake_controller_rate);
    if (message->position.size() == 1) {
      operateHand(pub_gripper_trajectory, message->position, duration);
    }
    if (message->position.size() == 2) {
      operateHead(pub_head_trajectory, message->position, duration);
    }
    if (message->position.size() == 5) {
      std::vector<double> positions = message->position;
      std::swap(positions[0], positions[1]); // Fix Order for Unity.
      moveArm(pub_arm_trajectory, positions, duration);
    }

  }

  void sendMessage(ros::Publisher &publisher, const std::string &message)
  {
    ROS_INFO("Send message:%s", message.c_str());

    handyman::HandymanMsg handyman_msg;
    handyman_msg.message = message;
    publisher.publish(handyman_msg);
  }

  tf::StampedTransform getTfBase(tf::TransformListener &tf_listener)
  {
    tf::StampedTransform tf_transform;

    try
    {
      tf_listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), tf_transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    return tf_transform;
  }

  void moveBase(ros::Publisher &publisher, double linear_x, double linear_y, double angular_z)
  {
    geometry_msgs::Twist twist;

    twist.linear.x  = linear_x;
    twist.linear.y  = linear_y;
    twist.angular.z = angular_z;
    publisher.publish(twist);
  }

  void stopBase(ros::Publisher &publisher)
  {
    moveBase(publisher, 0.0, 0.0, 0.0);
  }

  void moveArm(ros::Publisher &publisher, const std::vector<double> &positions, ros::Duration &duration)
  {
    arm_joint_trajectory_.points[0].positions = positions;
    arm_joint_trajectory_.points[0].time_from_start = duration;

    publisher.publish(arm_joint_trajectory_);
  }

  void operateHand(ros::Publisher &publisher, const std::vector<double> &positions, ros::Duration &duration)
  {
    std::vector<std::string> joint_names {"hand_motor_joint"};
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = duration;

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher.publish(joint_trajectory);
  }

  void initHead(){
    ros::Duration duration(1.25);
    std::vector<double> positions = {-0.5663, -0.8887};
    operateHead(pub_head_trajectory, positions, duration);
  }

  void operateHead(ros::Publisher &publisher, const std::vector<double> &positions, ros::Duration &duration)
  {
    std::vector<std::string> joint_names {"head_pan_joint", "head_tilt_joint"};
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = duration;

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher.publish(joint_trajectory);
  }

  void operateHand(ros::Publisher &publisher, bool should_grasp)
  {
    std::vector<std::string> joint_names {"hand_motor_joint"};
    std::vector<double> positions;

    if(should_grasp)
    {
      ROS_DEBUG("Grasp");
      positions.push_back(-0.105);
    }
    else
    {
      ROS_DEBUG("Open hand");
      positions.push_back(+1.239);
    }

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = ros::Duration(2);

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher.publish(joint_trajectory);
  }


public:

  HandymanMain(): 
    ac_detection3D(nh_, "Detect3D", true) 
  {
    ROS_INFO("Waiting for Detect3D action server to start.");
    ac_detection3D.waitForServer();
    ROS_INFO("Action server started, sending goal.");


    clear_octomap = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");
    clear_octomap.waitForExistence();
  }

  int run(int argc, char **argv)
  {

    ros::Rate loop_rate(10);

    std::string sub_msg_to_robot_topic_name;
    std::string pub_msg_to_moderator_topic_name;
    std::string pub_base_twist_topic_name;
    std::string pub_arm_trajectory_topic_name;
    std::string pub_gripper_trajectory_topic_name;
    std::string pub_head_trajectory_topic_name;
    std::string sub_trajectories_topic_name;

    nh_.param<std::string>("sub_msg_to_robot_topic_name",       sub_msg_to_robot_topic_name,       "/handyman/message/to_robot");
    nh_.param<std::string>("pub_msg_to_moderator_topic_name",   pub_msg_to_moderator_topic_name,   "/handyman/message/to_moderator");
    nh_.param<std::string>("pub_base_twist_topic_name",         pub_base_twist_topic_name,         "/hsrb/command_velocity");
    nh_.param<std::string>("pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
    nh_.param<std::string>("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_controller/command");
    nh_.param<std::string>("pub_head_trajectory_topic_name",    pub_head_trajectory_topic_name,    "/hsrb/head_trajectory_controller/command");
    nh_.param<std::string>("sub_trajectories_topic_name",       sub_trajectories_topic_name,       "/move_group/fake_controller_joint_states");

    init();

    ros::Time waiting_start_time;

    ROS_INFO("Handyman start!");

    ros::Subscriber sub_msg                = nh_.subscribe<handyman::HandymanMsg>(sub_msg_to_robot_topic_name, 100, &HandymanMain::messageCallback, this);
    ros::Publisher  pub_msg                = nh_.advertise<handyman::HandymanMsg>(pub_msg_to_moderator_topic_name, 10);
    ros::Publisher  pub_base_twist         = nh_.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
    pub_arm_trajectory     = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
    pub_gripper_trajectory = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);   
    pub_head_trajectory = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_head_trajectory_topic_name, 10);   
    ros::Subscriber sub_trajectories       = nh_.subscribe<sensor_msgs::JointState>(sub_trajectories_topic_name, 100, &HandymanMain::trajectoriesCallback, this);

    tf::TransformListener tf_listener;

    // Start: Temporary Added

    while (ros::ok())
    {
      if(is_failed_)
      {
        ROS_INFO("Task failed!");
        step_ = Initialize;
      }
      switch(step_)
      {
        case Initialize:
        {
          ROS_INFO("Initialize");
          reset();
          step_++;
          break;
        }
        case Ready:
        {
          if(is_started_)
          {
            ROS_INFO("Ready");
            
            // Set Unity Arm to Default Position.
            pub_arm_trajectory.publish(arm_joint_trajectory_);
            // Set Unity Head to Default Position.
            initHead();

            ros::Duration(2).sleep();
            std_srvs::Empty emptyCall;
            clear_octomap.call(emptyCall);

            sendMessage(pub_msg, MSG_I_AM_READY);

            ROS_INFO("Task start!");

            step_++;
          }
          break;
        }
        case WaitForInstruction:
        {
          if(instruction_msg_!="")
          {
            ROS_INFO("%s", instruction_msg_.c_str());

            // TODO: HANDLE INSTRUCTION

            step_++;
          }
          break;
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;

    // End: Temporary Added
    while (ros::ok())
    {
      if(is_failed_)
      {
        ROS_INFO("Task failed!");
        step_ = Initialize;
      }

      switch(step_)
      {
        case Initialize:
        {
          reset();
          step_++;
          break;
        }
        case Ready:
        {
          if(is_started_)
          {
            sendMessage(pub_msg, MSG_I_AM_READY);

            ROS_INFO("Task start!");

            step_++;
          }
          break;
        }
        case WaitForInstruction:
        {
          if(instruction_msg_!="")
          {
            ROS_INFO("%s", instruction_msg_.c_str());

            operateHand(pub_gripper_trajectory, false);

            step_++;
          }
          break;
        }
        case GoToRoom1:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() <= +0.2)
          {
            moveBase(pub_base_twist, +1.0, 0.0, 1.0);
          }
          else
          {
            stopBase(pub_base_twist);
            step_++;
          }

          break;
        }
        case GoToRoom2:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() <= +0.6)
          {
            moveBase(pub_base_twist, +1.0, 0.0, 0.0);
          }
          else
          {
            stopBase(pub_base_twist);
            sendMessage(pub_msg, MSG_ROOM_REACHED);

            step_++;
          }
          break;
        }
        case MoveToInFrontOfTarget:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() <= +1.0)
          {
            std::vector<double> positions { 0.22, -1.57, 0.0, 0.0, 0.0 };
            ros::Duration duration;
            duration.sec = 1;

            moveBase(pub_base_twist, +1.0, 0.0, 0.0);
            moveArm(pub_arm_trajectory, positions, duration);
          }
          else
          {
            stopBase(pub_base_twist);

            step_++;
          }

          break;
        }
        case Grasp:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() >= +1.2)
          {
            moveBase(pub_base_twist, +0.3, 0.0, 0.0);
          }
          else
          {
            stopBase(pub_base_twist);
            operateHand(pub_gripper_trajectory, true);

            waiting_start_time = ros::Time::now();
            step_++;
          }

          break;
        }
        case WaitForGrasping:
        {
          if(ros::Time::now() - waiting_start_time > ros::Duration(3, 0))
          {
            sendMessage(pub_msg, MSG_OBJECT_GRASPED);
            step_++;
          }

          break;
        }
        case ComeBack:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() <= 0.0)
          {
            moveBase(pub_base_twist, -1.0, 0.0, 0.0);
          }
          else
          {
            stopBase(pub_base_twist);
            sendMessage(pub_msg, MSG_TASK_FINISHED);

            step_++;
          }

          break;
        }
        case TaskFinished:
        {
          if(is_finished_)
          {
            ROS_INFO("Task finished!");
            step_ = Initialize;
          }

          break;
        }
      }

      ros::spinOnce();

      loop_rate.sleep();
    }

    return EXIT_SUCCESS;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "handyman_sample");

  HandymanMain handyman_sample;
  return handyman_sample.run(argc, argv);
};

