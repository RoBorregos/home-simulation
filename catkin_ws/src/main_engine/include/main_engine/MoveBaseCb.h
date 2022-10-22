#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace  move_base_msgs;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
