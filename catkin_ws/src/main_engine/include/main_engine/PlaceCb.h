#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_and_place/PickAndPlaceAction.h>

using namespace pick_and_place;
typedef actionlib::SimpleActionClient<PickAndPlaceAction> PickAndPlaceClient;

namespace PlaceCb{
  static bool active = true;
  static PickAndPlaceResultConstPtr result;
  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const PickAndPlaceResultConstPtr& result)
  {
    PlaceCb::result = result;
    ROS_INFO("Place - Finished in state [%s]", state.toString().c_str());
    PlaceCb::active = false;
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

  int execute(geometry_msgs::PoseStamped &target_pose, PickAndPlaceClient &ac_pick, tf::TransformListener &listener_)
  {
    tf::StampedTransform transform;
    listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
    tf::Quaternion currentRotation = transform.getRotation();
    target_pose.pose.orientation.x = currentRotation.getX();
    target_pose.pose.orientation.y = currentRotation.getY();
    target_pose.pose.orientation.z = currentRotation.getZ();
    target_pose.pose.orientation.w = currentRotation.getW();

    ROS_INFO("Executing Place");
    PickAndPlaceGoal goalPlace;
    goalPlace.target_pose = target_pose;
    goalPlace.object_name = "current";
    goalPlace.allow_contact_with = {"<octomap>"};
    PlaceCb::active = true;
    ac_pick.sendGoal(goalPlace, &PlaceCb::doneCb, &PlaceCb::activeCb, &PlaceCb::feedbackCb);
    ros::Rate loop_rate(10);
    while(PlaceCb::active) {
      loop_rate.sleep();
      continue;
    }
    ROS_INFO_STREAM("PLACE CODE: " << PlaceCb::result->error_code);
    return PlaceCb::result->error_code;
  }           
}

