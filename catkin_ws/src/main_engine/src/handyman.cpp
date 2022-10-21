#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <handyman/HandymanMsg.h>
#include <sensor_msgs/JointState.h>
#include <nodelet/nodelet.h>
#include <actionlib/client/simple_action_client.h>
#include <object_detector/DetectObjects3DAction.h>
#include <pick_and_place/PickAndPlaceAction.h>
#include <pick_and_place/EnableOctomap.h>
#include <gpd_ros/detect_grasps_samples.h>
#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <main_engine/Labels.h>
#include <chrono>
#include <thread>

using namespace object_detector;
using namespace pick_and_place;
using namespace  move_base_msgs;
typedef actionlib::SimpleActionClient<DetectObjects3DAction> Detect3DClient;
typedef actionlib::SimpleActionClient<PickAndPlaceAction> PickAndPlaceClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

namespace Detect3DCb{
  static bool active = true;
  static DetectObjects3DResultConstPtr result;
  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const DetectObjects3DResultConstPtr& result)
  {
    Detect3DCb::result = result;
    ROS_INFO("Detect3D - Finished in state [%s]", state.toString().c_str());
    Detect3DCb::active = false;
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Detect3D - Goal just went active");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const DetectObjects3DFeedbackConstPtr& feedback)
  {
    ROS_INFO("Detect3D - Got Feedback");
  }
}

namespace PickCb{
  static bool active = true;
  static PickAndPlaceResultConstPtr result;
  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const PickAndPlaceResultConstPtr& result)
  {
    PickCb::result = result;
    ROS_INFO("Pick - Finished in state [%s]", state.toString().c_str());
    PickCb::active = false;
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Pick - Goal just went active");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const PickAndPlaceFeedbackConstPtr& feedback)
  {
    ROS_INFO("Pick - Got Feedback");
  }
}

namespace PlaceCb{
  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const PickAndPlaceResultConstPtr& result)
  {
    ROS_INFO("Place - Finished in state [%s]", state.toString().c_str());
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Place - Goal just went active");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const PickAndPlaceFeedbackConstPtr& feedback)
  {
    ROS_INFO("Place - Got Feedback");
  }
}

namespace MoveBaseCb{
  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const MoveBaseResultConstPtr& result)
  {
    ROS_INFO("MoveBase - Finished in state [%s]", state.toString().c_str());
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("MoveBase - Goal just went active");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const MoveBaseActionFeedbackConstPtr& feedback)
  {
    ROS_INFO("MoveBase - Got Feedback");
  }
}

bool allTrue(std::vector<bool> values) {
  for(auto value: values) {
    if (value == false) {
      return false;
    }
  }
  return true;
}

class HandymanMain
{
private:
  enum Step
  {
    Initialize,
    Ready,
    WaitForInstruction,
    GoToRoom1,
    Grasp,
    Place,
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
  int force_step_;

  std::string instruction_msg_;

  bool is_started_;
  bool is_finished_;
  bool is_failed_;

  ros::Publisher  pub_arm_trajectory;
  ros::Publisher  pub_base_trajectory;
  ros::Publisher  pub_gripper_trajectory;
  ros::Publisher  pub_head_trajectory;
  ros::Publisher  pub_base_twist;

  tf::TransformListener listener_;

  uint64_t valid_till = 0.0;
  bool is_moving = false;

  // clear_octomap
  ros::ServiceClient clear_octomap;
  ros::ServiceClient enable_octomap;


  ros::NodeHandle nh_;
  Detect3DClient ac_detection3D; 
  PickAndPlaceClient ac_pick;
  PickAndPlaceClient ac_place;
  MoveBaseClient ac_move;

  moveit::planning_interface::MoveGroupInterface* move_arm_;
  moveit::planning_interface::MoveGroupInterface* move_head_;
  moveit::planning_interface::MoveGroupInterface* move_hand_;
  moveit::planning_interface::MoveGroupInterface* move_all_;

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
    std::vector<std::string> arm_joint_names {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
    std::vector<double> arm_joint_positions(arm_joint_names.size(), 0.0);
    std::vector<bool> arm_joint_filled(arm_joint_names.size(), false);
    std::vector<std::string> base_joint_names {"odom_r", "odom_x", "odom_y"};
    std::vector<double> base_joint_positions(base_joint_names.size(), 0.0);
    std::vector<bool> base_joint_filled(base_joint_names.size(), false);
    std::vector<std::string> hand_joint_names {"hand_motor_joint"};
    std::vector<double> hand_joint_positions(hand_joint_names.size(), 0.0);
    std::vector<bool> hand_joint_filled(hand_joint_names.size(), false);
    std::vector<std::string> head_joint_names {"head_pan_joint", "head_tilt_joint"};
    std::vector<double> head_joint_positions(head_joint_names.size(), 0.0);
    std::vector<bool> head_joint_filled(head_joint_names.size(), false);

    for (int i=0;i<message->name.size();i++) {
      for(int j=0;j<arm_joint_names.size();j++) {
        if (arm_joint_names[j] == message->name[i]) {
          arm_joint_positions[j] = message->position[i];
          arm_joint_filled[j] = true;
        }
      }
      for(int j=0;j<base_joint_names.size();j++) {
        if (base_joint_names[j] == message->name[i]) {
          base_joint_positions[j] = message->position[i];
          base_joint_filled[j] = true;
        }
      }
      for(int j=0;j<hand_joint_names.size();j++) {
        if (hand_joint_names[j] == message->name[i]) {
          hand_joint_positions[j] = message->position[i];
          hand_joint_filled[j] = true;
        }
      }
      for(int j=0;j<head_joint_names.size();j++) {
        if (head_joint_names[j] == message->name[i]) {
          head_joint_positions[j] = message->position[i];
          head_joint_filled[j] = true;
        }
      }
    }
    if (allTrue(hand_joint_filled)) {
      operateHand(pub_gripper_trajectory, hand_joint_positions, hand_joint_names, duration);
    }
    if (allTrue(head_joint_filled)) {
      operateHead(pub_head_trajectory, head_joint_positions, head_joint_names, duration);
    }
    if (allTrue(arm_joint_filled)) {
      moveArm(pub_arm_trajectory, arm_joint_positions, arm_joint_names, duration);
    }
    if (allTrue(base_joint_filled)) {
      operateBase(pub_base_trajectory, base_joint_positions);
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

  void stopBase(ros::Publisher &publisher)
  {
    moveBase(publisher, 0.0, 0.0, 0.0, 1000);
    is_moving = false;
    valid_till = 0.0;
  }

  void checkBase() {
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      if (is_moving && timeSinceEpochMillisec() > valid_till) {
        stopBase(pub_base_twist);
      }
      loop_rate.sleep();
    }
  }

  void moveArm(ros::Publisher &publisher, const std::vector<double> &positions, std::vector<std::string> &joint_names, ros::Duration &duration)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = duration;

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher.publish(joint_trajectory);
  }

  void operateHand(ros::Publisher &publisher, const std::vector<double> &positions, std::vector<std::string> &joint_names, ros::Duration &duration)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = duration;

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher.publish(joint_trajectory);
  }

  bool moveHead(const std::vector<double> &positions){
    move_head_->setJointValueTarget(positions);
    move_head_->setPlanningTime(1.0);
    bool success = (move_head_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO_STREAM("Head moved to initial position.");
      return true;
    }
    ROS_INFO_STREAM("Head movement to initial position failed.");
    return false;
  }

  bool moveHand(const std::vector<double> &positions){
    move_hand_->setJointValueTarget(positions);
    move_hand_->setPlanningTime(1.0);
    bool success = (move_hand_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO_STREAM("Hand moved to initial position.");
      return true;
    }
    ROS_INFO_STREAM("Hand movement to initial position failed.");
    return false;
  }

  bool moveArm(const std::vector<double> &positions){
    move_arm_->setJointValueTarget(positions);
    move_arm_->setPlanningTime(1.0);
    bool success = (move_arm_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO_STREAM("Arm moved to initial position.");
      return true;
    }
    ROS_INFO_STREAM("Arm movement to initial position failed.");
    return false;
  }

  bool moveAll(){
    geometry_msgs::PoseStamped position;
    position.header.frame_id = "map";
    position.pose.position.x = 0.5524308294591512;
    position.pose.position.y = 0.24833975277601641;
    position.pose.position.z = 0.7068511427706418; 
    position.pose.orientation.x = -0.5195918821618419;
    position.pose.orientation.y = 0.5197011721605078;
    position.pose.orientation.z = -0.025166215400788592;
    position.pose.orientation.w = 0.6777179570063863;

    move_all_->setPoseTarget(position);
    bool success = (move_all_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO_STREAM("All moved to target position.");
      return true;
    }
    ROS_INFO_STREAM("All movement to target position failed.");
    return false;
  }

  void operateHead(ros::Publisher &publisher, const std::vector<double> &positions, std::vector<std::string> &joint_names, ros::Duration &duration)
  {
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = duration;

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher.publish(joint_trajectory);
  }

  void operateBase(ros::Publisher &publisher, const std::vector<double> &positions)
  {
    static double last_r = 0;
    static double last_x = 0;
    static double last_y = 0;
    static int64_t last_t = 0;

    if (last_t == 0 || timeSinceEpochMillisec() - last_t > 500) {
      last_r = positions[0];
      last_x = positions[1];
      last_y = positions[2];
      last_t = timeSinceEpochMillisec();
      return;
    }

    double delta_t = timeSinceEpochMillisec() - last_t;
    double delta_r = 1000 * ((positions[0] - last_r) / delta_t);
    double delta_x = 1000 * ((positions[1] - last_x) / delta_t);
    double delta_y = 1000 * ((positions[2] - last_y) / delta_t);
    
    moveBase(pub_base_twist, delta_x, delta_y, delta_r, delta_t);

    last_r = positions[0];
    last_x = positions[1];
    last_y = positions[2];
    last_t = timeSinceEpochMillisec();
    // std::vector<std::string> joint_names {"odom_r", "odom_x", "odom_y"};

    // trajectory_msgs::JointTrajectoryPoint point;
    // point.positions = positions;
    // point.time_from_start = duration;

    // trajectory_msgs::JointTrajectory joint_trajectory;
    // joint_trajectory.joint_names = joint_names;
    // joint_trajectory.points.push_back(point);
    // publisher.publish(joint_trajectory);
  }

  void moveBase(ros::Publisher &publisher, double linear_x, double linear_y, double angular_z, double time_ms)
  {
    if(listener_.canTransform("/odom_xr_link", "/base_footprint", ros::Time(0)) == false)
    {
      return;
    }

    geometry_msgs::PointStamped basefootprint_2_target;
    geometry_msgs::PointStamped odom_2_target;
    basefootprint_2_target.header.frame_id = "/odom_xr_link";
    basefootprint_2_target.header.stamp = ros::Time(0);
    basefootprint_2_target.point.x = linear_x;
    basefootprint_2_target.point.y = linear_y;
    listener_.transformPoint("/base_footprint", basefootprint_2_target, odom_2_target);

    tf::StampedTransform transform;
    listener_.lookupTransform("/odom_xr_link", "/base_footprint", ros::Time(0), transform);
    tf::Quaternion currentRotation = transform.getRotation();
    tf::Matrix3x3 mat(currentRotation);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);


    valid_till = timeSinceEpochMillisec() + time_ms;
    geometry_msgs::Twist twist;

    twist.linear.x  = odom_2_target.point.x;
    twist.linear.y  = odom_2_target.point.y;
    twist.angular.z = angular_z;
    publisher.publish(twist);

    is_moving = true;
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
    point.time_from_start = ros::Duration(5);

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher.publish(joint_trajectory);
  }

  bool moveRobot(geometry_msgs::PoseStamped target) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = target;

    ROS_INFO("Sending goal");
    ac_move.sendGoal(goal);
    ac_move.waitForResult();

    if(ac_move.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Move Robot Successfull");
      return true;
    }
    
    ROS_INFO("Move Robot Failed");
    return false;
  }

  void rotateTowardsPoint(geometry_msgs::Point point_) {
    tf::TransformListener tf_listener;
    tf::StampedTransform tf_transform = getTfBase(tf_listener);
    tf::Vector3 dest(point_.x, point_.y, point_.z);
    tf::Vector3 origin(tf_transform.getOrigin().x(), tf_transform.getOrigin().y(), tf_transform.getOrigin().z());
    origin.normalize();
    dest.normalize();
    geometry_msgs::Quaternion q;
    tf::Vector3 res = origin.cross(dest);
    q.x = res.getX();
    q.y = res.getY();
    q.z = res.getZ();
    q.w = sqrt((origin.length() * origin.length()) * (dest.length() * dest.length())) + origin.dot(dest);

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "map";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = origin.getX();
    target_pose.pose.position.y = origin.getY();
    target_pose.pose.position.z = origin.getZ();
    target_pose.pose.orientation = q;

    moveRobot(target_pose);
  }

public:

  HandymanMain(): 
    ac_detection3D(nh_, "Detect3D", true),
    ac_pick(nh_, "/pickup_pose", true),
    ac_place(nh_, "/place_pose", true),
    ac_move(nh_, "/move_base", true)
  {
    ROS_INFO("Waiting for Detect3D action server to start.");
    ac_detection3D.waitForServer();
    ROS_INFO("Waiting for Pick action server to start.");
    ac_pick.waitForServer();
    ROS_INFO("Waiting for Place action server to start.");
    ac_place.waitForServer();
    // ROS_INFO("Waiting for Move Base action server to start.");
    // ac_move.waitForServer();

    ROS_INFO("Waiting for Clear Octomap service to start.");
    clear_octomap = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");
    clear_octomap.waitForExistence();
    enable_octomap = nh_.serviceClient<pick_and_place::EnableOctomap>("/enable_octomap");
    enable_octomap.waitForExistence();

    ROS_INFO("Ready!");

  }

  int run(int argc, char **argv)
  {

    ros::Rate loop_rate(10);

    std::string sub_msg_to_robot_topic_name;
    std::string pub_msg_to_moderator_topic_name;
    std::string pub_base_twist_topic_name;
    std::string pub_arm_trajectory_topic_name;
    std::string pub_base_trajectory_topic_name;
    std::string pub_gripper_trajectory_topic_name;
    std::string pub_head_trajectory_topic_name;
    std::string sub_trajectories_topic_name;

    nh_.param<std::string>("/handyman/sub_msg_to_robot_topic_name",       sub_msg_to_robot_topic_name,       "/handyman/message/to_robot");
    nh_.param<std::string>("/handyman/pub_msg_to_moderator_topic_name",   pub_msg_to_moderator_topic_name,   "/handyman/message/to_moderator");
    nh_.param<std::string>("/handyman/pub_base_twist_topic_name",         pub_base_twist_topic_name,         "/hsrb/command_velocity");
    nh_.param<std::string>("/handyman/pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
    nh_.param<std::string>("/handyman/pub_base_trajectory_topic_name",    pub_base_trajectory_topic_name,    "/hsrb/omni_base_controller/command");
    nh_.param<std::string>("/handyman/pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_controller/command");
    nh_.param<std::string>("/handyman/pub_head_trajectory_topic_name",    pub_head_trajectory_topic_name,    "/hsrb/head_trajectory_controller/command");
    nh_.param<std::string>("/handyman/sub_trajectories_topic_name",       sub_trajectories_topic_name,       "/move_group/fake_controller_joint_states");
    nh_.param<int>("/handyman/force_step", force_step_, -1);


    init();

    ros::Time waiting_start_time;

    ROS_INFO("Handyman start!");

    moveit::planning_interface::MoveGroupInterface move_arm("arm"); move_arm_ = &move_arm;
    moveit::planning_interface::MoveGroupInterface move_head("head"); move_head_ = &move_head;
    moveit::planning_interface::MoveGroupInterface move_hand("hand"); move_hand_ = &move_hand;
    moveit::planning_interface::MoveGroupInterface move_all("whole_body"); move_all_ = &move_all;
    ros::Subscriber sub_msg                = nh_.subscribe<handyman::HandymanMsg>(sub_msg_to_robot_topic_name, 100, &HandymanMain::messageCallback, this);
    ros::Publisher  pub_msg                = nh_.advertise<handyman::HandymanMsg>(pub_msg_to_moderator_topic_name, 10);
    pub_base_twist         = nh_.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
    pub_arm_trajectory     = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
    pub_base_trajectory     = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_base_trajectory_topic_name, 10);
    pub_gripper_trajectory = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);   
    pub_head_trajectory = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_head_trajectory_topic_name, 10);   
    ros::Subscriber sub_trajectories       = nh_.subscribe<sensor_msgs::JointState>(sub_trajectories_topic_name, 100, &HandymanMain::trajectoriesCallback, this);
    ros::ServiceClient client_grasping     = nh_.serviceClient<gpd_ros::detect_grasps_samples>("/detect_grasps_server_samples/detect_grasps_samples");
    ROS_INFO("Waiting for Client Grasping Service!");
    client_grasping.waitForExistence();

    tf::TransformListener tf_listener;

    // Start: Temporary Added
    ROS_INFO_STREAM("Force Step: " << force_step_);
    std::thread t1(&HandymanMain::checkBase, this);
    while (ros::ok() && force_step_ == 100)
    {

      ros::spinOnce();
      loop_rate.sleep();
    }
    while (ros::ok())
    {
      if(is_failed_)
      {
        ROS_INFO("Task failed!");
        step_ = Initialize;
      }
      if(force_step_ != -1) {
        step_ = force_step_;
        force_step_ = -1;
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

            std_srvs::Empty emptyCall;
            clear_octomap.call(emptyCall);

            // Set Unity Head to Default Position.
            ROS_INFO("HEAD Default Position");
            moveHead(vector<double>({0.0, -0.6887}));
            // Set Unity Hand to Default Position.
            ROS_INFO("HAND Default Position - OPEN");
            moveHand(vector<double>({1.0598}));
            // Set Unity Arm to Default Position.
            ROS_INFO("ARM Default Position");
            moveArm(vector<double>({ 0.0, 0.0, 1.5708, -1.5708, 0.0 }));

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

            step_++;
            step_++; // TODO: EnableGoToRoom.
          }
          break;
        }
        case GoToRoom1:
        {
          if (moveRobot(NavPosesDict[MAP::L20_01][ROOM::LIVING_ROOM][PLACE::SQUARE_LOW_TABLE].val)){
            
            step_++;
          }
          break;
        }
        case Grasp:
        {
          bool pickFailed = false;
          int status_code = -1;
          int attempts = 3;

          while (status_code < 0 && attempts > 0) {
            std_srvs::Empty emptyCall;
            clear_octomap.call(emptyCall);
            pick_and_place::EnableOctomap srvEOctomap;
            srvEOctomap.request.enable = true;
            enable_octomap.call(srvEOctomap);

            // Check Left.
            ROS_INFO("HEAD Left Position");
            moveHead(vector<double>({0.6, -0.6887}));
            // Check Right.
            ROS_INFO("HEAD Right Position");
            moveHead(vector<double>({-0.6, -0.6887}));
            // Default.
            ROS_INFO("HEAD Default Position");
            moveHead(vector<double>({0.0, -0.6887}));

            // srvEOctomap.request.enable = false;
            // enable_octomap.call(srvEOctomap);

            DetectObjects3DGoal goal;
            ROS_INFO("Executing 3D Vision");
            Detect3DCb::active = true;
            ac_detection3D.sendGoal(goal, &Detect3DCb::doneCb, &Detect3DCb::activeCb, &Detect3DCb::feedbackCb);
            while(Detect3DCb::active) {
              loop_rate.sleep();
              continue;
            }
            if (Detect3DCb::result == NULL || Detect3DCb::result->object_cloud.samples.size() == 0) {
              ROS_INFO_STREAM("No object found");
              step_++;
              break;
            }

            gpd_ros::detect_grasps_samples srv_grasps;
            srv_grasps.request.cloud_samples = Detect3DCb::result->object_cloud;
            ROS_INFO_STREAM("Result Samples: " << srv_grasps.request.cloud_samples.samples.size());
            if (client_grasping.call(srv_grasps)) {
              gpd_ros::GraspConfigList grasp_configs = srv_grasps.response.grasp_configs;
              if (grasp_configs.grasps.size() != 0) {
                ROS_INFO_STREAM("Received grasps: " << grasp_configs.grasps.size());
                PickAndPlaceGoal goalPick;
                goalPick.grasp_config_list = grasp_configs;
                goalPick.object_name = "current";
                goalPick.allow_contact_with = {"<octomap>"}; // TODO: Add table.
                PickCb::active = true;
                ac_pick.sendGoal(goalPick, &PickCb::doneCb, &PickCb::activeCb, &PickCb::feedbackCb);
                ROS_INFO_STREAM("Grasps Sended.");
                while(PickCb::active) {
                  loop_rate.sleep();
                  continue;
                }
                attempts -= 1;
                status_code = PickCb::result->error_code;
                if (status_code == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED || status_code == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN) {
                  attempts = 0;
                }
                if (status_code == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                  ROS_INFO_STREAM("Grasp SUCCESSFULL");
                }else {
                  ROS_INFO_STREAM("Grasp Failed. Remaining Attempts" << attempts);  
                }
              } else {
                ROS_INFO_STREAM("No grasps received");
                step_++;
                break;
              }
            }
            else {
              ROS_ERROR("Failed to call service client_grasping");
            }
          }
          step_++;
          break;
        }
        case Place:
        {
          PickAndPlaceGoal goalPlace;
          geometry_msgs::PoseStamped* target_pose = &goalPlace.target_pose;
          target_pose->header.frame_id = "map";
          target_pose->header.stamp = ros::Time::now();
          target_pose->pose.position.x = 0.4280264973640442;
          target_pose->pose.position.y = 0.5202085375785828;
          target_pose->pose.position.z = 0.5050736665725708 + 0.1; // Add Table Margin
          tf::StampedTransform transform;
          listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
          tf::Quaternion currentRotation = transform.getRotation();
          target_pose->pose.orientation.x = currentRotation.getX();
          target_pose->pose.orientation.y = currentRotation.getY();
          target_pose->pose.orientation.z = currentRotation.getZ();
          target_pose->pose.orientation.w = currentRotation.getW();

          goalPlace.object_name = "current";
          goalPlace.allow_contact_with = {"<octomap>"}; // TODO: Add table.
          ac_place.sendGoal(goalPlace, &PlaceCb::doneCb, &PlaceCb::activeCb, &PlaceCb::feedbackCb);
          ROS_INFO_STREAM("Place Sended.");
          step_++;
          break;
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "handyman_sample");
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  HandymanMain handyman_sample;
  return handyman_sample.run(argc, argv);
};

