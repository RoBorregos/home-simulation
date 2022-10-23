#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace  move_base_msgs;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace MoveBaseCb{
  static bool active = true;
  static MoveBaseResultConstPtr result;

  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const MoveBaseResultConstPtr& result)
  {
    MoveBaseCb::result = result;
    ROS_INFO("MoveBase - Finished in state [%s]", state.toString().c_str());
    MoveBaseCb::active = false;
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("MoveBase - Goal just went active");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const MoveBaseFeedbackConstPtr& feedback)
  {
    ROS_INFO("MoveBase - Got Feedback");
  }

  bool execute(geometry_msgs::PoseStamped target, MoveBaseClient &ac_move) {
    int attempts = 3;
    int state = 0;
    while(state != actionlib::SimpleClientGoalState::SUCCEEDED && attempts-- > 0) {
      ROS_INFO("Moving Base");
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose = target;
      MoveBaseCb::active = true;
      ac_move.sendGoal(goal, &MoveBaseCb::doneCb, &MoveBaseCb::activeCb, &MoveBaseCb::feedbackCb);
      ros::Rate loop_rate(10);
      while(MoveBaseCb::active) {
        loop_rate.sleep();
        continue;
      }
      state = ac_move.getState().state_;
    }
    return state == actionlib::SimpleClientGoalState::SUCCEEDED;
  }
}
