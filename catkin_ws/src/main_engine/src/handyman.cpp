#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <handyman/HandymanMsg.h>
#include <sensor_msgs/JointState.h>
#include <nodelet/nodelet.h>
#include <actionlib/client/simple_action_client.h>
#include <object_detector/DetectObjects3DAction.h>
#include <object_detector/objectDetectionArray.h>
#include <object_detector/objectDetection.h>
#include <pick_and_place/PickAndPlaceAction.h>
#include <pick_and_place/EnableOctomap.h>
#include <gpd_ros/detect_grasps_samples.h>
#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/LoadMap.h>
#include <main_engine/Labels.h>
#include <main_engine/MoveBaseCb.h>
#include <main_engine/ParserCb.h>
#include <main_engine/Detect3DCb.h>
#include <main_engine/PickCb.h>
#include <main_engine/PlaceCb.h>
#include <chrono>
#include <thread>

using namespace object_detector;
using namespace pick_and_place;
using namespace  move_base_msgs;
typedef actionlib::SimpleActionClient<PickAndPlaceAction> PickAndPlaceClient;

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

bool allTrue(std::vector<bool> values) {
  for(auto value: values) {
    if (value == false) {
      return false;
    }
  }
  return true;
}

const bool PARSER_ENABLE = false;
const bool DETECTION_ENABLE = false;
const bool MOVE_BASE_ENABLE = false;

class HandymanMain
{
private:
  enum Step
  {
    Initialize,
    Ready,
    WaitForInstruction,
    GoToRoom1,
    Exploration,
    Grasp,
    GoToRoom2,
    Place,
    ComeBack,
    TaskFinished,
    GiveUp,
    Wait,
  };

  int step_;
  int force_step_;

  std::string instruction_msg_;

  string currEnvironment = "";
  string newEnvironment = "";
  bool is_started_;
  bool is_new_;
  bool is_finished_;
  bool is_failed_;

  ros::Publisher  pub_active_2D;
  ros::Publisher  pub_arm_trajectory;
  ros::Publisher  pub_base_trajectory;
  ros::Publisher  pub_gripper_trajectory;
  ros::Publisher  pub_head_trajectory;
  ros::Publisher  pub_base_twist;
  ros::Publisher  pub_initial_pose;
  ros::Publisher  pub_info_pose;

  tf::TransformListener listener_;

  uint64_t valid_till = 0.0;
  bool is_moving = false;
  TaskInfo currentTask;
  OBJECT targetDetection;
  bool targetDetectionFound = false;
  vector<double> headTargetDetectionFound = vector<double>({0.0, -0.5887});

  // clear_octomap
  ros::ServiceClient clear_octomap;
  ros::ServiceClient enable_octomap;

  ros::NodeHandle nh_;
  Detect3DClient ac_detection3D; 
  PickAndPlaceClient ac_pick;
  ParserClient ac_parser;
  PickAndPlaceClient ac_place;
  MoveBaseClient ac_move;

  moveit::planning_interface::MoveGroupInterface* move_arm_;
  moveit::planning_interface::MoveGroupInterface* move_head_;
  moveit::planning_interface::MoveGroupInterface* move_hand_;
  moveit::planning_interface::MoveGroupInterface* move_all_;

  void init()
  {
    step_ = Initialize;
    reset();
  }

  void reset()
  {
    instruction_msg_ = "";
    is_started_  = false;
    is_new_ = true;
    is_finished_ = false;
    is_failed_   = false;
    disable2D();
  }


  void detectionsCallback(const object_detector::objectDetectionArray::ConstPtr& msg)
  {
    for (auto detection : msg->detections) {
      if (detection.labelText == objectsr[targetDetection]) {
        targetDetectionFound = true;
      }
    }
  }

  void messageCallback(const handyman::HandymanMsg::ConstPtr& message)
  {
    ROS_INFO("Subscribe message:%s, %s", message->message.c_str(), message->detail.c_str());

    if(message->message.c_str()==MSG::ARE_YOU_READY)
    {
      if(step_==Ready)
      {
        is_started_ = true;
      }
    }
    if(message->message.c_str()==MSG::ENVIRONMENT)
    {
      newEnvironment = message->detail;
    }
    if(message->message.c_str()==MSG::INSTRUCTION)
    {
      if(step_==WaitForInstruction)
      {
        instruction_msg_ = message->detail.c_str();
      }
    }
    if(message->message.c_str()==MSG::TASK_SUCCEEDED)
    {
      if(step_==TaskFinished)
      {
        is_finished_ = true;
      }
    }
    if(message->message.c_str()==MSG::TASK_FAILED)
    {
      is_failed_ = true;
    }
    if(message->message.c_str()==MSG::MISSION_COMPLETE)
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
    std::map<std::string, double> positions_({
      {"arm_flex_joint", positions[0]},
      {"arm_lift_joint", positions[1]},
      {"arm_roll_joint", positions[2]},
      {"wrist_flex_joint", positions[3]},
      {"wrist_roll_joint", positions[4]}
    }); 
    move_arm_->setJointValueTarget(positions_);
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
  }

  void transformMapToCamera(geometry_msgs::Point &value) {
    geometry_msgs::PointStamped old;
    geometry_msgs::PointStamped newp;
    old.header.frame_id = "/map";
    old.header.stamp = ros::Time(0);
    old.point = value;
    listener_.transformPoint("/head_rgbd_sensor_depth_frame", old, newp);
    value = newp.point;
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
    publishInfo(target);
    return MoveBaseCb::execute(target, ac_move);
  }

  void publishInfo(geometry_msgs::PoseStamped target) {
    return pub_info_pose.publish(target);
    geometry_msgs::PoseStamped position;
    position.header.frame_id = "map";
    position.pose.position.x = 0.5524308294591512;
    position.pose.position.y = 0.24833975277601641;
    position.pose.position.z = 0.7068511427706418; 
    position.pose.orientation.x = -0.5195918821618419;
    position.pose.orientation.y = 0.5197011721605078;
    position.pose.orientation.z = -0.025166215400788592;
    position.pose.orientation.w = 0.6777179570063863;
  }

  void publishInfo(geometry_msgs::Point target) {
    geometry_msgs::PoseStamped position;
    position.header.frame_id = "map";
    position.pose.position.x = target.x; 
    position.pose.position.y = target.y; 
    position.pose.position.z = target.z; 
    position.pose.orientation.x = 0.0;
    position.pose.orientation.y = 0.0;
    position.pose.orientation.z = 0.0;
    position.pose.orientation.w = 1;
    return pub_info_pose.publish(position);
  }

  void setInitialPose() {
    std::string fixed_frame = "map";
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp=ros::Time::now();
    pose.pose.pose.position.x = 0;
    pose.pose.pose.position.y = 0;
    pose.pose.pose.position.z = 0;
    pose.pose.pose.orientation.x = 0;
    pose.pose.pose.orientation.y = 0;
    pose.pose.pose.orientation.z = 0;
    pose.pose.pose.orientation.w = 1;
    pub_initial_pose.publish(pose);
  }

  void changeMap() {
    std_srvs::Empty emptyCall;
    clear_octomap.call(emptyCall);

    if (newEnvironment != "Layout2020HM01" && newEnvironment != "Layout2019HM01" &&
      newEnvironment != "Layout2021HM01" && newEnvironment != "Layout2019HM02") {
      ROS_INFO_STREAM("INVALID MAP");
    } else {
      nav_msgs::LoadMap::Request  req;
      nav_msgs::LoadMap::Response resp;
      req.map_url = ros::package::getPath("nav_main") + "/maps/"+newEnvironment+".yaml";
      ros::service::waitForService("change_map", 5000);
      ros::service::call("change_map", req, resp);
      ros::Duration(4).sleep();
      setInitialPose(); 
      ros::Duration(2).sleep();
    }
  }

  void enable2D() {
    std_msgs::Bool val;val.data = true;
    pub_active_2D.publish(std_msgs::Bool(val));    
  }
  void disable2D() {
    std_msgs::Bool val;val.data = false;
    pub_active_2D.publish(std_msgs::Bool(val));    
  }

  bool explorePlace(NavPose targetPose) {
    if (moveRobot(targetPose.val)){
      // Enable 2D.
      targetDetection = currentTask.GRASP_OBJ;
      targetDetectionFound = false;
      enable2D();
      ros::Duration(3).sleep();

      if (targetDetectionFound) {
        disable2D();
        step_=Grasp;
        headTargetDetectionFound = vector<double>({0.0 -0.5887});
        return true;
      }
      // Check Left.
      ROS_INFO("HEAD Left Position");
      moveHead(vector<double>({0.5, -0.5887}));
      if (targetDetectionFound) {
        disable2D();
        headTargetDetectionFound = vector<double>({0.5, -0.5887});
        step_=Grasp;
        return true;
      }
      // Check Right.
      ROS_INFO("HEAD Right Position");
      moveHead(vector<double>({-0.5, -0.5887}));
      if (targetDetectionFound) {
        disable2D();
        headTargetDetectionFound = vector<double>({-0.5, -0.5887});
        step_=Grasp;
        return true;
      }
      // Default.
      ROS_INFO("HEAD Default Position");
      moveHead(vector<double>({0.0, -0.5887}));
      disable2D();
    }
    return false;
  }

public:

  HandymanMain(): 
    ac_detection3D(nh_, "Detect3D", true),
    ac_parser(nh_, "Parser", true),
    ac_pick(nh_, "/pickup_pose", true),
    ac_place(nh_, "/place_pose", true),
    ac_move(nh_, "/move_base", true)
  {
    ROS_INFO("Waiting for Detect3D action server to start.");
    ac_detection3D.waitForServer();
    ROS_INFO("Waiting for Pick action server to start.");
    ac_pick.waitForServer();
    ROS_INFO("Waiting for Parser action server to start.");
    if (PARSER_ENABLE) {
      ac_parser.waitForServer();
    }
    ROS_INFO("Waiting for Place action server to start.");
    ac_place.waitForServer();
    ROS_INFO("Waiting for Move Base action server to start.");
    if (MOVE_BASE_ENABLE) {
      ac_move.waitForServer();
    }

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

    ros::Time waiting_start_time;

    ROS_INFO("Handyman start!");

    moveit::planning_interface::MoveGroupInterface move_arm("arm"); move_arm_ = &move_arm;
    moveit::planning_interface::MoveGroupInterface move_head("head"); move_head_ = &move_head;
    moveit::planning_interface::MoveGroupInterface move_hand("hand"); move_hand_ = &move_hand;
    moveit::planning_interface::MoveGroupInterface move_all("whole_body"); move_all_ = &move_all;
    ros::Subscriber sub_msg = nh_.subscribe<handyman::HandymanMsg>(sub_msg_to_robot_topic_name, 100, &HandymanMain::messageCallback, this);
    ros::Subscriber sub_detections = nh_.subscribe<object_detector::objectDetectionArray>("/detections", 100, &HandymanMain::detectionsCallback, this);
    ros::Publisher  pub_msg = nh_.advertise<handyman::HandymanMsg>(pub_msg_to_moderator_topic_name, 10);
    pub_initial_pose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
    pub_info_pose = nh_.advertise<geometry_msgs::PoseStamped>("/info/pose", 1000);
    pub_active_2D = nh_.advertise<std_msgs::Bool>("detectionsActive", 10);
    pub_base_twist = nh_.advertise<geometry_msgs::Twist>(pub_base_twist_topic_name, 10);
    pub_arm_trajectory = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
    pub_base_trajectory = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_base_trajectory_topic_name, 10);
    pub_gripper_trajectory = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);   
    pub_head_trajectory = nh_.advertise<trajectory_msgs::JointTrajectory>(pub_head_trajectory_topic_name, 10);   
    ros::Subscriber sub_trajectories = nh_.subscribe<sensor_msgs::JointState>(sub_trajectories_topic_name, 100, &HandymanMain::trajectoriesCallback, this);
    ros::ServiceClient client_grasping = nh_.serviceClient<gpd_ros::detect_grasps_samples>("/detect_grasps_server_samples/detect_grasps_samples");
    ROS_INFO("Waiting for Client Grasping Service!");
    client_grasping.waitForExistence();

    tf::TransformListener tf_listener;

    init();

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
          step_=Ready;
          break;
        }
        case GiveUp:
        {
          ROS_INFO("Give Up");
          sendMessage(pub_msg, MSG::GIVE_UP);
          step_=Wait;
          break;
        }
        case Ready:
        {
          if(is_started_)
          {
            if (is_new_) {
              ros::Duration(1).sleep();
              is_new_ = false;
            }
            
            if (currEnvironment != newEnvironment) {
              ROS_INFO("Changing Map!");
              changeMap();
              currEnvironment = newEnvironment;
            }

            ROS_INFO("Ready");

            // Set Unity Head to Default Position.
            ROS_INFO("HEAD Default Position");
            moveHead(vector<double>({0.0, -0.5887}));
            // Set Unity Hand to Default Position.
            ROS_INFO("HAND Default Position - OPEN");
            moveHand(vector<double>({1.0598}));
            // Set Unity Arm to Default Position.
            ROS_INFO("ARM Default Position");
            moveArm(vector<double>({ -0.088, 0.191, 1.5708, -1.5708, 0.0 }));

            sendMessage(pub_msg, MSG::I_AM_READY);
            ROS_INFO("Task start!");

            step_=WaitForInstruction;
          }
          break;
        }
        case WaitForInstruction:
        {
          if(instruction_msg_!="")
          {
            ROS_INFO("%s", instruction_msg_.c_str());
            currentTask = ParserCb::execute(instruction_msg_, ac_parser, PARSER_ENABLE);
            if (currentTask.GO_TO == ROOM::DEFAULT_ROOM) {
              step_=GiveUp;
            } else {
              step_=GoToRoom1;
            }
          }
          break;
        }
        case GoToRoom1:
        {
          ROS_INFO("GoToRoom1.");
          MAP currentMap = maps[currEnvironment];
          ROOM targetRoom = currentTask.GO_TO;
          PLACE targetPlace = PLACE::SAFE_PLACE;
          NavPose targetPose;
          map<PLACE, NavPose>* dictROOM = &NavPosesDict[currentMap][targetRoom];
          if (dictROOM->find(targetPlace) == dictROOM->end()) {
            ROS_INFO_STREAM("Missing definition for " << mapsr[currentMap] << ":" << roomsr[targetRoom] << ":" << placesr[targetPlace]);
            if (dictROOM->size() == 0) {
              ROS_INFO_STREAM("Missing definition for " << mapsr[currentMap] << ":" << roomsr[targetRoom]);
              step_=GiveUp;
              break;
            } else {
              targetPose = (dictROOM->begin()->second);
            }
          } else {
            targetPose = (*dictROOM)[targetPlace];
          }
          if (moveRobot(targetPose.val)){
            sendMessage(pub_msg, MSG::ROOM_REACHED);
            step_=Exploration;
          } else {
            ROS_INFO_STREAM("Move Failed, Giving Up.");
            step_=GiveUp;
          }
          break;
        }
        case Exploration:
        {
          ROS_INFO("Exploration.");
          MAP currentMap = maps[currEnvironment];
          ROOM targetRoom = currentTask.GO_TO;
          map<PLACE, NavPose>* dictROOM = &NavPosesDict[currentMap][targetRoom];

          // Prioritize Places
          if (currentTask.GRASP_REF1_PLACE != PLACE::DEFAULT_PLACE) {
            if ( (*dictROOM).find(currentTask.GRASP_REF1_PLACE) != (*dictROOM).end()){
              bool result = explorePlace((*dictROOM)[currentTask.GRASP_REF1_PLACE]);
              if (result){break;}
            }
          }
          if (currentTask.GRASP_REF2_PLACE != PLACE::DEFAULT_PLACE) {
            if ( (*dictROOM).find(currentTask.GRASP_REF2_PLACE) != (*dictROOM).end()){
              bool result = explorePlace((*dictROOM)[currentTask.GRASP_REF2_PLACE]);
              if (result){break;}
            }
          }

          // Look in all other places in the room.
          bool result = false;
          for(auto roomPlace : *(dictROOM)) {
            if (roomPlace.first == currentTask.GRASP_REF1_PLACE ||
            roomPlace.first == currentTask.GRASP_REF2_PLACE){
              continue;
            }
            bool result = explorePlace(roomPlace.second);
            if (result){break;}
          }
          if (result){break;}
          // Object not found case.
          step_=WaitForInstruction;
          sendMessage(pub_msg, MSG::DOES_NOT_EXIST);
          break;
        }
        case Grasp:
        {
          ROS_INFO("Grasp");
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
            moveHead(vector<double>({0.6, -0.5887}));
            // Check Right.
            ROS_INFO("HEAD Right Position");
            moveHead(vector<double>({-0.6, -0.5887}));
            // Default.
            ROS_INFO("HEAD Found Object Position");
            moveHead(headTargetDetectionFound);

            // Use best out of 3 Detections.
            enable2D();
            boost::shared_ptr<object_detector::objectDetectionArray const> input_detections = ros::topic::waitForMessage<object_detector::objectDetectionArray>("/detections", nh_);
            boost::shared_ptr<object_detector::objectDetectionArray const> input_detections2 = ros::topic::waitForMessage<object_detector::objectDetectionArray>("/detections", nh_);
            boost::shared_ptr<object_detector::objectDetectionArray const> input_detections3 = ros::topic::waitForMessage<object_detector::objectDetectionArray>("/detections", nh_);
            input_detections = input_detections->detections.size() < input_detections2->detections.size() ? input_detections2 : input_detections;
            input_detections = input_detections->detections.size() < input_detections3->detections.size() ? input_detections3 : input_detections;
            disable2D();
            
            object_detector::objectDetectionArray force_object;
            for (auto detection : input_detections->detections) {
              if (detection.labelText == objectsr[currentTask.GRASP_OBJ]) {
                publishInfo(detection.point3D);
                if (!DETECTION_ENABLE) {
                  detection.point3D.z+=0.05; // Add value on z to avoid table pieces.
                  transformMapToCamera(detection.point3D);
                }
                force_object.detections.push_back(detection);
                break;
              }
            }
            Detect3DCb::execute(ac_detection3D, force_object);
            if (Detect3DCb::result == NULL || Detect3DCb::result->object_cloud.samples.size() == 0) {
              ROS_INFO_STREAM("No object found");
              attempts -= 1;continue;
            }

            gpd_ros::detect_grasps_samples srv_grasps;
            srv_grasps.request.cloud_samples = Detect3DCb::result->object_cloud;
            if (client_grasping.call(srv_grasps)) {
              gpd_ros::GraspConfigList grasp_configs = srv_grasps.response.grasp_configs;
              if (grasp_configs.grasps.size() != 0) {
                status_code = PickCb::execute(grasp_configs, ac_pick);
                attempts -= 1;
                if (status_code == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED ||
                  status_code == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN) {
                  attempts = 0;
                  continue;
                }
                if (status_code == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                  ROS_INFO_STREAM("GRASP SUCCESS");
                }else {
                  ROS_INFO_STREAM("GRASP FAILED: #" << attempts);  
                }
              } else {
                ROS_INFO_STREAM("No grasps received");
                attempts -= 1;
              }
            }
            else {
              ROS_ERROR("Failed to call service client_grasping");
              attempts -= 1;
            }
          }
          if (status_code == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            step_=GoToRoom2;          
            sendMessage(pub_msg, MSG::OBJECT_GRASPED);
          } else {
            step_=GiveUp;
          }
          break;
        }
        case GoToRoom2:
        {
          ROS_INFO("GoToRoom2");
          MAP currentMap = maps[currEnvironment];
          ROOM targetRoom = currentTask.PUT_ROOM;
          PLACE destination = currentTask.PUT_PLACE;

          if (targetRoom != ROOM::DEFAULT_ROOM) {
            if (NavPosesDict[currentMap][targetRoom].find(destination) != NavPosesDict[currentMap][targetRoom].end()) {
              moveRobot(NavPosesDict[currentMap][targetRoom][destination].val);
              step_=Place;
              break;
            }
          }
          bool found = false;
          for(auto navPosesRoom: NavPosesDict[currentMap]) {
            if (found){break;}
            for(auto navPosesPlace: navPosesRoom.second) {
              if (navPosesPlace.first != destination){continue;}
              currentTask.PUT_ROOM = navPosesRoom.first;
              moveRobot(navPosesPlace.second.val);
              step_=Place;
              found = true;
              break;
            }
          }

          if (found){break;}
          step_=GiveUp;
          break;
        }
        case Place:
        {
          ROS_INFO("Place");          
          MAP currentMap = maps[currEnvironment];
          ROOM targetRoom = currentTask.PUT_ROOM;
          PLACE destination = currentTask.PUT_PLACE;

          int status_code = -1;
          int attempts = 3;

          if (targetRoom != ROOM::DEFAULT_ROOM) {
            if (PlacePosesDict[currentMap][targetRoom].find(destination) != PlacePosesDict[currentMap][targetRoom].end()) {
              vector<ObjectPlaceInfo> objectplaces = PlacePosesDict[currentMap][targetRoom][destination];
              // TO-DO Order to prioritize REFs.
              for(auto objectplace: objectplaces) {
                geometry_msgs::PoseStamped target_pose = objectplace.val.val;
                publishInfo(target_pose);
                target_pose.pose.position.z = 0.5050736665725708 + 0.1; // Add Table Margin
                
                while (status_code < 0 && attempts > 0) {
                  status_code = PlaceCb::execute(target_pose, ac_pick, listener_);
                  attempts -= 1;
                  if (status_code == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED ||
                    status_code == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN) {
                    attempts = 0;
                    continue;
                  }
                  if (status_code == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                    ROS_INFO_STREAM("GRASP SUCCESS");
                  }else {
                    ROS_INFO_STREAM("GRASP FAILED: #" << attempts);  
                  }
                }
                if (status_code == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                  step_=TaskFinished;          
                  sendMessage(pub_msg, MSG::TASK_FINISHED);
                  break;
                }
              }
              step_=Place;
              break;
            }
          }
              
          step_=GiveUp;
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

