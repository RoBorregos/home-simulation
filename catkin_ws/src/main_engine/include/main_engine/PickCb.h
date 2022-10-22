#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_and_place/PickAndPlaceAction.h>

using namespace pick_and_place;
typedef actionlib::SimpleActionClient<PickAndPlaceAction> PickAndPlaceClient;

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

  int execute(gpd_ros::GraspConfigList &grasp_configs, PickAndPlaceClient &ac_pick)
  {
    ROS_INFO("Executing Pick");
    PickAndPlaceGoal goalPick;
    goalPick.grasp_config_list = grasp_configs;
    goalPick.object_name = "current";
    goalPick.allow_contact_with = {"<octomap>"};
    PickCb::active = true;
    ac_pick.sendGoal(goalPick, &PickCb::doneCb, &PickCb::activeCb, &PickCb::feedbackCb);
    ros::Rate loop_rate(10);
    while(PickCb::active) {
      loop_rate.sleep();
      continue;
    }
    ROS_INFO_STREAM("PICK CODE: " << PickCb::result->error_code);
    return PickCb::result->error_code;
  }           
}

