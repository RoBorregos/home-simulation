#ifndef NAV_POSES_H_
#define NAV_POSES_H_

#include <geometry_msgs/Pose.h>
#include <map>

using namespace std;

enum MAP {
  TEST1,
  TEST2,
  TEST3,
  JAPAN2022_1
};

enum ROOM {
  KITCHEN,
  BEDROOM,
  LIVING_ROOM,
  LOBBY
};

enum PLACE {
  SAFE_PLACE,
  LOW_TABLE,
  WAGON,
  WHITE_TABLE,
  TRASH_BOX_FOR_RECYCLE,
  TRASH_BOX_FOR_BUMABLE,
  TRASH_BOX_FOR_BOTTLE_CAN,
};

struct NavPose
{
  NavPose(){}
  NavPose(float px, float py, float pz, float ox, float oy, float oz, float ow){
    val.header.frame_id = "map";
    val.pose.position.x = px;
    val.pose.position.y = py;
    val.pose.position.z = pz;
    val.pose.orientation.x = ox;
    val.pose.orientation.y = oy;
    val.pose.orientation.z = oz;
    val.pose.orientation.w = ow;
  } 
  geometry_msgs::PoseStamped val;
};

map<MAP, map<ROOM, map<PLACE, NavPose>>> NavPosesDict ({
  {
    MAP::TEST1,
    {
      {
        ROOM::LIVING_ROOM, {
          {PLACE::SAFE_PLACE, NavPose(2.09310, 0.18493, 0.0, 0.0, 0.0, 0.87606, -0.48218)},
          {PLACE::LOW_TABLE, NavPose(2.09310, 0.18493, 0.0, 0.0, 0.0, 0.87606, -0.48218)},
        }
      },
      {
        ROOM::BEDROOM, {
          {PLACE::LOW_TABLE, NavPose(2.09310, 0.18493, 0.0, 0.0, 0.0, 0.87606, -0.48218)}
        }
      },
    }
  }
});

#endif /* NAV_POSES_H_ */
