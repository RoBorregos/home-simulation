#include <ros/service.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/LoadMap.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

boost::shared_ptr<nav_msgs::OccupancyGrid const> map_;
std_msgs::String mapID;
void mapCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map){
    map_ = map;
}

 void mapIDCallback(const std_msgs::String::ConstPtr& msg)
   {
     mapID.data = msg->data.c_str();
   }
int main(int argc, char **argv)
{
  bool enter = true;
  ros::init(argc, argv, "Roborregos_map_change");

  ros::NodeHandle n;
  ros::ServiceClient clear_costmap;

  ros::Subscriber sub_map = n.subscribe("map", 1000, mapCallback);
  ros::Subscriber sub_mapID = n.subscribe("mapID", 1000, mapIDCallback);
  clear_costmap = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    //Receive Type of map
    if(mapID.data=="LayoutA" && enter == true){
      std_srvs::Empty emptyCall;
      ros::service::waitForService("/move_base/clear_costmaps", 10000);
      clear_costmap.call(emptyCall);
      // Now change the map
      nav_msgs::LoadMap::Request  req;
      nav_msgs::LoadMap::Response resp;
      req.map_url = ros::package::getPath("nav_main") + "/maps/roborregosmapB.yaml";
      ros::service::waitForService("change_map", 5000);
      ros::service::call("change_map", req, resp);
      //Put the old map back so the next test isn't broken
      //req.map_url = ros::package::getPath("nav_main") + "/maps/roborregosmap.yaml";
      //ros::service::call("change_map", req, resp);
      enter = false;
    }
    else if(mapID.data=="LayoutB"){
        //Change map
    }
    else if(mapID.data=="LayoutC"){
        //Change map
    }
    else if(mapID.data=="LayoutD"){
        //Change map
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}