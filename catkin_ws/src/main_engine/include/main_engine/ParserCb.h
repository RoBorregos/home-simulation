#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <main_engine/ParserAction.h>
#include <main_engine/Labels.h>

using namespace main_engine;
typedef actionlib::SimpleActionClient<ParserAction> ParserClient;

struct TaskInfo
{
  TaskInfo(){}
  TaskInfo(Entities ent):
    ent_(ent)
  {
    GO_TO = rooms[ent.GO_TO];
    GRASP_OBJ = objects[ent.GRASP_OBJ]; 
    GRASP_PREP = prepositions[ent.GRASP_PREP]; 
    GRASP_REF1_PLACE = places[ent.GRASP_REF1]; 
    GRASP_REF1_OBJECT = objects[ent.GRASP_REF1]; 
    GRASP_REF2_PLACE = places[ent.GRASP_REF2]; 
    GRASP_REF2_OBJECT = objects[ent.GRASP_REF2]; 
    PUT_PLACE = places[ent.PUT_PLACE];
    PUT_ROOM = rooms[ent.PUT_ROOM];
    PUT_PREP = prepositions[ent.PUT_PREP]; 
    PUT_REF1_PLACE = places[ent.PUT_REF1]; 
    PUT_REF1_OBJECT = objects[ent.PUT_REF1]; 
    PUT_REF2_PLACE = places[ent.PUT_REF2]; 
    PUT_REF2_OBJECT = objects[ent.PUT_REF2]; 
  }
  Entities ent_;
  ROOM GO_TO;
  OBJECT GRASP_OBJ;
  PREPOSITION GRASP_PREP;
  PLACE GRASP_REF1_PLACE;
  OBJECT GRASP_REF1_OBJECT;
  PLACE GRASP_REF2_PLACE;
  OBJECT GRASP_REF2_OBJECT;
  PLACE PUT_PLACE;
  ROOM PUT_ROOM;
  PREPOSITION PUT_PREP;
  PLACE PUT_REF1_PLACE;
  OBJECT PUT_REF1_OBJECT;
  PLACE PUT_REF2_PLACE;
  OBJECT PUT_REF2_OBJECT;
};

namespace ParserCb{
  static bool active = true;
  static ParserResultConstPtr result;

  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const ParserResultConstPtr& result)
  {
    ParserCb::result = result;
    ROS_INFO("Parser - Finished in state [%s]", state.toString().c_str());
    ParserCb::active = false;
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Parser - Goal just went active");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const ParserFeedbackConstPtr& feedback)
  {
    ROS_INFO("Parser - Got Feedback");
  }

  TaskInfo execute(string msg, ParserClient &ac_parser, bool PARSER_ENABLE)
  {
    TaskInfo res;
    if (!PARSER_ENABLE) {
      res.GO_TO = LIVING_ROOM;
      res.GRASP_OBJ = SUGAR;
      res.GRASP_PREP = ON_THE;
      res.GRASP_REF1_PLACE = WHITE_SIDE_TABLE;
      res.PUT_PLACE = SOFA;
      res.PUT_ROOM = LIVING_ROOM;
    }
    int attempts = 3;
    while(PARSER_ENABLE && res.GO_TO == ROOM::DEFAULT_ROOM && attempts-- > 0) {
      ROS_INFO("Executing Parser");
      ParserGoal goalParser;
      goalParser.instruction = msg;
      ParserCb::active = true;
      ac_parser.sendGoal(goalParser, &ParserCb::doneCb, &ParserCb::activeCb, &ParserCb::feedbackCb);
      ros::Rate loop_rate(10);
      while(ParserCb::active) {
        loop_rate.sleep();
        continue;
      }
      res = TaskInfo(ParserCb::result->entities);
    }
    ROS_INFO_STREAM("Parser Finished");

    ROS_INFO_STREAM("GO_TO: " << res.GO_TO); 
    ROS_INFO_STREAM("GRASP_OBJ: " << res.GRASP_OBJ); 
    ROS_INFO_STREAM("GRASP_PREP: " << res.GRASP_PREP); 
    ROS_INFO_STREAM("GRASP_REF1_PLACE: " << res.GRASP_REF1_PLACE); 
    ROS_INFO_STREAM("GRASP_REF1_OBJECT: " << res.GRASP_REF1_OBJECT); 
    ROS_INFO_STREAM("GRASP_REF2_PLACE: " << res.GRASP_REF2_PLACE); 
    ROS_INFO_STREAM("GRASP_REF2_OBJECT: " << res.GRASP_REF2_OBJECT); 
    ROS_INFO_STREAM("PUT_PLACE: " << res.PUT_PLACE); 
    ROS_INFO_STREAM("PUT_PREP: " << res.PUT_PREP); 
    ROS_INFO_STREAM("PUT_REF1_PLACE: " << res.PUT_REF1_PLACE); 
    ROS_INFO_STREAM("PUT_REF1_OBJECT: " << res.PUT_REF1_OBJECT); 
    ROS_INFO_STREAM("PUT_REF2_PLACE: " << res.PUT_REF2_PLACE); 
    ROS_INFO_STREAM("PUT_REF2_OBJECT: " << res.PUT_REF2_OBJECT); 
    return res;
  }
}
