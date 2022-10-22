#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <object_detector/DetectObjects3DAction.h>

using namespace object_detector;
typedef actionlib::SimpleActionClient<DetectObjects3DAction> Detect3DClient;

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

  void execute(Detect3DClient &ac_detection3D)
  {
    ros::Rate loop_rate(10);
    DetectObjects3DGoal goal;
    ROS_INFO("Executing 3D Vision");
    Detect3DCb::active = true;
    ac_detection3D.sendGoal(goal, &Detect3DCb::doneCb, &Detect3DCb::activeCb, &Detect3DCb::feedbackCb);
    while(Detect3DCb::active) {
      loop_rate.sleep();
      continue;
    }
  }
}
