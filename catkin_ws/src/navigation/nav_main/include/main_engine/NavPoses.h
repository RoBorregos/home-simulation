#ifndef NAV_POSES_H_
#define NAV_POSES_H_

#include <geometry_msgs/Pose.h>
#include <map>

using namespace std;

enum MAP {
  LAYOUTA,
  LAYOUTB,
  LAYOUTC,
  LAYOUTD,
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
  PersonM
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
    MAP::LAYOUTA,
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
      {
        ROOM::LIVING_ROOM, {
          {PLACE::PersonM, NavPose(-0.5808882713317871, -0.09954459965229034, 0.0, 0.0, 0.0, 0.9999827411436484, 0.005875152324416911)},
          {PLACE::WHITE_TABLE, NavPose(2.0541205406188965, 1.960193157196045, 0.0, 0.0, 0.0, -0.7104442764459433, 0.7037534582971512)},
          {PLACE::TRASH_BOX_FOR_BOTTLE_CAN, NavPose()},
          {PLACE::TRASH_BOX_FOR_BUMABLE, NavPose()},
          {PLACE::TRASH_BOX_FOR_RECYCLE, NavPose()},
          {PLACE::LOW_TABLE, NavPose()},
        }
      },
    }
  }
});

#endif /* NAV_POSES_H_ */