#include <ros/service.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/LoadMap.h>
#include <std_msgs/String.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

boost::shared_ptr<nav_msgs::OccupancyGrid const> map_;
std_msgs::String mapID;
bool newmap=false;
void mapCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map){
    map_ = map;
}

 void mapIDCallback(const std_msgs::String::ConstPtr& msg)
   {
    if(msg->data!=mapID.data){
      mapID.data = msg->data.c_str();
      newmap=true;
    } 
     
   }
int main(int argc, char **argv)
{
  //bool enter = true;
  ros::init(argc, argv, "Roborregos_map_change");

  ros::NodeHandle n;
  ros::ServiceClient clear_costmap;

  ros::Subscriber sub_map = n.subscribe("map", 1000, mapCallback);
  ros::Subscriber sub_mapID = n.subscribe("mapID", 1000, mapIDCallback);
  ros::Publisher  set_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
  std::string fixed_frame = "map";
  geometry_msgs::PoseWithCovarianceStamped pose;


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    //Receive Type of map
    if(mapID.data=="Layout2020HM01" && newmap == true){
      // Now change the map
      nav_msgs::LoadMap::Request  req;
      nav_msgs::LoadMap::Response resp;
      req.map_url = ros::package::getPath("nav_main") + "/maps/Layout2020HM01.yaml";
      ros::service::waitForService("change_map", 5000);
      ros::service::call("change_map", req, resp);
      pose.header.frame_id = fixed_frame;
      pose.header.stamp=ros::Time::now();
      pose.pose.pose.position.x = 0;
      pose.pose.pose.position.y = 0;
      pose.pose.pose.position.z = 0;
      pose.pose.pose.orientation.x = 0;
      pose.pose.pose.orientation.y = 0;
      pose.pose.pose.orientation.z = -0.017841180377059222;
      pose.pose.pose.orientation.w = 0.9998408334743851;
      set_pose_pub.publish(pose);
      newmap = false;
    }
    else if(mapID.data=="Layout2019HM01" && newmap == true){
        //Change map
      nav_msgs::LoadMap::Request  req;
      nav_msgs::LoadMap::Response resp;
      req.map_url = ros::package::getPath("nav_main") + "/maps/Layout2019HM01.yaml";
      ros::service::waitForService("change_map", 5000);
      ros::service::call("change_map", req, resp);
      pose.header.frame_id = fixed_frame;
      pose.header.stamp=ros::Time::now();
      pose.pose.pose.position.x = 0;
      pose.pose.pose.position.y = 0;
      pose.pose.pose.position.z = 0;
      pose.pose.pose.orientation.x = 0;
      pose.pose.pose.orientation.y = 0;
      pose.pose.pose.orientation.z = -0.017841180377059222;
      pose.pose.pose.orientation.w = 0.9998408334743851;
      set_pose_pub.publish(pose);
      newmap = false;
    }
    else if(mapID.data=="Layout2021HM01" && newmap == true){
        //Change map
        nav_msgs::LoadMap::Request  req;
        nav_msgs::LoadMap::Response resp;
        req.map_url = ros::package::getPath("nav_main") + "/maps/Layout2021HM01.yaml";
        ros::service::waitForService("change_map", 5000);
        ros::service::call("change_map", req, resp);
        pose.header.frame_id = fixed_frame;
        pose.header.stamp=ros::Time::now();
        pose.pose.pose.position.x = 0;
        pose.pose.pose.position.y = 0;
        pose.pose.pose.position.z = 0;
        pose.pose.pose.orientation.x = 0;
        pose.pose.pose.orientation.y = 0;
        pose.pose.pose.orientation.z = -0.017841180377059222;
        pose.pose.pose.orientation.w = 0.9998408334743851;
        set_pose_pub.publish(pose);
    }
    else if(mapID.data=="Layout2019HM02" && newmap == true){
        //Change map
        nav_msgs::LoadMap::Request  req;
        nav_msgs::LoadMap::Response resp;
        req.map_url = ros::package::getPath("nav_main") + "/maps/Layout2019HM02.yaml";
        ros::service::waitForService("change_map", 5000);
        ros::service::call("change_map", req, resp);
        pose.header.frame_id = fixed_frame;
        pose.header.stamp=ros::Time::now();
        pose.pose.pose.position.x = 0;
        pose.pose.pose.position.y = 0;
        pose.pose.pose.position.z = 0;
        pose.pose.pose.orientation.x = 0;
        pose.pose.pose.orientation.y = 0;
        pose.pose.pose.orientation.z = -0.017841180377059222;
        pose.pose.pose.orientation.w = 0.9998408334743851;
        set_pose_pub.publish(pose);
        newmap = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}