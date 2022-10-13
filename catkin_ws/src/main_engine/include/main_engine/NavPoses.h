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
  NavPose(float px, float py, float pz, float ox, float oy, float oz, float ow){
    p.position.x = px;
    p.position.y = py;
    p.position.z = pz;
    p.orientation.x = ox;
    p.orientation.y = oy;
    p.orientation.z = oz;
    p.orientation.w = ow;
  } 
  geometry_msgs::Pose p;
};

map<MAP, map<ROOM, map<PLACE, NavPose>>> NavPosesDict ({
  {
    MAP::TEST1,
    {
      {
        ROOM::KITCHEN, {
          {PLACE::SAFE_PLACE, NavPose(1.9607, 0.07947, 0.0, 1.67, 1.3, -0.79, 0.606)},
          {PLACE::LOW_TABLE, NavPose(1.9607, 0.07947, 0.0, 1.67, 1.3, -0.79, 0.606)},
        }
      },
      {
        ROOM::BEDROOM, {
          {PLACE::LOW_TABLE, NavPose(1.9607, 0.07947, 0.0, 1.67, 1.3, -0.79, 0.606)}
        }
      },
    }
  }
});

#endif /* NAV_POSES_H_ */
