#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

#include <object_detector/objectDetectionArray.h>

#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/CloudSources.h>

#include <actionlib/server/simple_action_server.h>
#include <object_detector/DetectObjects3DAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>


#include <std_msgs/Int64.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <limits>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#define CAMERA_FRAME "head_rgbd_sensor_depth_frame"

using namespace octomap;

struct ObjectParams
{
  /* PointCloud Cluster. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
  /* PointCloud Cluster Original. */
  pcl::PointCloud<pcl::PointXYZ> cluster_original;
  /* Mesh. */
  shape_msgs::Mesh::Ptr mesh;
  /* Center point of the Cluster. */
  geometry_msgs::PoseStamped center;
  /* Center point of the Cluster referenced to Camera TF. */
  geometry_msgs::PoseStamped center_cam;
  /* World Z Min Value. */
  double min_z;
  /* World Z Max Value. */
  double max_z;
  /* World X Min Value. */
  double min_x;
  /* World X Max Value. */
  double max_x;
  /* World Y Min Value. */
  double min_y;
  /* World Y Max Value. */
  double max_y;
  /* Detection Label. */
  int label = -1;
};

class Detect3D
{
  const std::string name = "Detect3D";
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  actionlib::SimpleActionServer<object_detector::DetectObjects3DAction> as_;
  object_detector::DetectObjects3DFeedback feedback_;
  object_detector::DetectObjects3DResult result_;
  ros::Publisher pose_pub_;
  ros::Publisher gpd_pub_;
  geometry_msgs::PoseArray pose_pub_msg_;
  gpd_ros::CloudSamples gpd_pub_msg_;
  ros::Publisher pc_pub_1_;
  ros::Publisher pc_pub_2_;
  ros::ServiceClient clear_octomap;
public:
  Detect3D() :
    listener_(buffer_),
    as_(nh_, name, boost::bind(&Detect3D::handleActionServer, this, _1), false)
  {
    tf_listener = new tf::TransformListener();
    as_.start();
    pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/test/objectposes", 10);
    gpd_pub_ = nh_.advertise<gpd_ros::CloudSamples>("/cloud_stitched", 10);
    pc_pub_1_ = nh_.advertise<sensor_msgs::PointCloud2>("/test_pc_1", 10);
    pc_pub_2_ = nh_.advertise<sensor_msgs::PointCloud2>("/test_pc_2", 10);
    ROS_INFO("Waiting for Clear Octomap service to start.");
    clear_octomap = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");
    clear_octomap.waitForExistence();
    ROS_INFO_STREAM("Action Server Detect3D - Initialized");
  }

  /** \brief Handle Action Server Goal Received. */
  void handleActionServer(const object_detector::DetectObjects3DGoalConstPtr &goal)
  {
    ROS_INFO_STREAM("Action Server Detect3D - Goal Received");
    feedback_.status = 0;
    result_.objects_found = 0;
    result_.objects_poses.clear();
    result_.objects_names.clear();
    result_.objects_ids.clear();
    result_.x_plane = 0;
    result_.y_plane = 0;
    result_.z_plane = 0;
    result_.width_plane = 0;
    result_.height_plane = 0;
    pose_pub_msg_.poses.clear();

    gpd_pub_msg_.cloud_sources = gpd_ros::CloudSources();
    gpd_pub_msg_.samples.clear();

    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO_STREAM("Action Server Detect3D - Preempted");
      as_.setPreempted(); // Set the action state to preempted
      return;
    }

    // Get PointCloud and Transform it to Map Frame
    sensor_msgs::PointCloud2 pc;
    sensor_msgs::PointCloud2 t_pc;
    tf_listener->waitForTransform("/map", CAMERA_FRAME, ros::Time(0), ros::Duration(5.0));
    pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points", nh_));
    pc.header.frame_id = CAMERA_FRAME;
    pcl_ros::transformPointCloud("map", pc, t_pc, *tf_listener);

    Detect3D::cloudCB(t_pc);
    as_.setSucceeded(result_);
    pose_pub_.publish(pose_pub_msg_);
  }

  /** \brief Given the parameters of the object add it to the planning scene. */
  void addObject(std::string id, const ObjectParams& object_found, const ObjectParams &table_params, bool checkTable = true)
  {
    // Adding Object to Planning Scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "map";
    collision_object.id = id;

    geometry_msgs::PoseStamped object_pose;
    object_pose.header.stamp = ros::Time::now();
    object_pose.header.frame_id = "map";
    object_pose.pose.position = object_found.center.pose.position;
    object_pose.pose.orientation = object_found.center.pose.orientation;

    // Add object only if is above table.
    if (checkTable && object_pose.pose.position.z < table_params.min_z) {
      return;
    }

    collision_object.meshes.push_back(*object_found.mesh);
    collision_object.mesh_poses.push_back(object_pose.pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface_.applyCollisionObject(collision_object);

    result_.objects_found++;
    pose_pub_msg_.header = object_pose.header;
    pose_pub_msg_.poses.push_back(object_pose.pose);
    result_.objects_poses.push_back(object_pose);
    result_.objects_names.push_back(collision_object.id);
    result_.objects_ids.push_back(object_found.label);
  }

  /** \brief Given the parameters of the plane add it to the planning scene. */
  bool addPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::ModelCoefficients::Ptr& coefficients_plane, ObjectParams &plane_params)
  {
    extractObjectDetails(cloud, plane_params, false);

    // Z conditions to know if it is the Table.
    if (plane_params.max_z - plane_params.min_z > 0.1) { // Diff > 10cm, Not Parallel Plane 
      return false;
    }
    if (plane_params.min_z < 0.20) { // Z Value < than 20cm, Floor Detected.
      return false;
    }

    // Adding Plane to Planning Scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "map";
    collision_object.id = "table";

    // Define a cylinder which will be added to the world.
    shape_msgs::SolidPrimitive solid_primitive;
    solid_primitive.type = solid_primitive.BOX;
    solid_primitive.dimensions.resize(3);
    solid_primitive.dimensions[0] = std::abs(plane_params.max_y - plane_params.min_y) * 2/3;
    solid_primitive.dimensions[1] = std::abs(plane_params.max_x - plane_params.min_x) * 2/3;
    solid_primitive.dimensions[2] = 0.02; // Custom table Z.

    geometry_msgs::PoseStamped plane_pose;
    plane_pose.header.stamp = ros::Time::now();
    plane_pose.header.frame_id = "map";
    plane_pose.pose.position.x = (plane_params.max_x + plane_params.min_x) / 2;
    plane_pose.pose.position.y = (plane_params.max_y + plane_params.min_y) / 2;
    plane_pose.pose.position.z = (plane_params.max_z + plane_params.min_z) / 2 - 0.04;
    ROS_INFO_STREAM("Plane Detected at " << std::to_string((plane_params.max_z + plane_params.min_z) / 2));
    plane_pose.pose.orientation.x = 0.0; //atan2(fabs(plane_params.max_x_p.y - plane_params.min_x_p.y), fabs(plane_params.max_x_p.x - plane_params.min_x_p.x));
    plane_pose.pose.orientation.y = 0.0; //atan2(fabs(plane_params.max_y_p.y - plane_params.min_y_p.y), fabs(plane_params.max_y_p.x - plane_params.min_y_p.x));
    plane_pose.pose.orientation.z = 0.0; 
    plane_pose.pose.orientation.w = 1;

    collision_object.primitives.push_back(solid_primitive);
    collision_object.primitive_poses.push_back(plane_pose.pose);
    collision_object.operation = collision_object.ADD;
    // TODO: Improve Plane Definition or conitnue using octomap.
    // planning_scene_interface_.applyCollisionObject(collision_object);

    return true;
  }

  template<typename T>
  void toPoint(const T &in, geometry_msgs::Point &out)
  {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
  }

  /** \brief Given pcl Polygon Mesh convert it to ros Mesh.
      @param polygon_mesh_ptr - Polygon Mesh Pointer. */
  bool convertPolygonMeshToRosMesh(const pcl::PolygonMesh::Ptr polygon_mesh_ptr, shape_msgs::Mesh::Ptr ros_mesh_ptr) {
    ROS_INFO("Conversion from PCL PolygonMesh to ROS Mesh started.");

    pcl_msgs::PolygonMesh pcl_msg_mesh;

    pcl_conversions::fromPCL(*polygon_mesh_ptr, pcl_msg_mesh);

    sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

    size_t size = pcd_modifier.size();

    ros_mesh_ptr->vertices.resize(size);

    ROS_INFO_STREAM("polys: " << pcl_msg_mesh.polygons.size()
                              << " vertices: " << pcd_modifier.size());

    sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

    for (size_t i = 0u; i < size; i++, ++pt_iter) {
      ros_mesh_ptr->vertices[i].x = pt_iter[0];
      ros_mesh_ptr->vertices[i].y = pt_iter[1];
      ros_mesh_ptr->vertices[i].z = pt_iter[2];
    }

    ROS_INFO_STREAM("Updated vertices");

    ros_mesh_ptr->triangles.resize(polygon_mesh_ptr->polygons.size());

    for (size_t i = 0u; i < polygon_mesh_ptr->polygons.size(); ++i) {
      if (polygon_mesh_ptr->polygons[i].vertices.size() < 3u) {
        ROS_WARN_STREAM("Not enough points in polygon. Ignoring it.");
        continue;
      }

      for (size_t j = 0u; j < 3u; ++j) {
        ros_mesh_ptr->triangles[i].vertex_indices[j] =
            polygon_mesh_ptr->polygons[i].vertices[j];
      }
    }
    ROS_INFO("Conversion from PCL PolygonMesh to ROS Mesh ended.");
    return true;
  }

  /** \brief Given the pointcloud containing just the object, reconstruct a mesh from it.
      @param cloud - point cloud containing just the object. */
  void reconstructMesh(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, shape_msgs::Mesh::Ptr &mesh)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.02); // 2cm

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(*triangles);

    convertPolygonMeshToRosMesh(triangles, mesh);
  }

  /** \brief Given the pointcloud containing just the object,
      compute its center point, its height and its mesh and store in object_found.
      @param cloud - point cloud containing just the object. */
  void extractObjectDetails(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, ObjectParams& object_found, bool getMesh = true)
  {
    ROS_INFO_STREAM("Extracting Object Details " << cloud->points.size());
    double max_z_ = cloud->points[0].z;
    double min_z_ = cloud->points[0].z;
    double max_y_ = cloud->points[0].y;
    double min_y_ = cloud->points[0].y;
    double max_x_ = cloud->points[0].x;
    double min_x_ = cloud->points[0].x;
    for (auto const point : cloud->points)
    {
      max_z_ = fmax(max_z_, point.z);
      min_z_ = fmin(min_z_, point.z);
      max_x_ = fmax(max_x_, point.x);
      min_x_ = fmin(min_x_, point.x);
      max_y_ = fmax(max_y_, point.y);
      min_y_ = fmin(min_y_, point.y);
    }
    object_found.max_z = max_z_;
    object_found.min_z = min_z_;
    object_found.max_x = max_x_;
    object_found.min_x = min_x_;
    object_found.max_y = max_y_;
    object_found.min_y = min_y_;

    pcl::PointXYZ centroid;
    pcl::computeCentroid(*cloud, centroid);

    // Store the object centroid.
    object_found.center.header.frame_id = "map";
    object_found.center.pose.position.x = centroid.x;
    object_found.center.pose.position.y = centroid.y;
    object_found.center.pose.position.z = centroid.z;
    object_found.center.pose.orientation.x = 0.0;
    object_found.center.pose.orientation.y = 0.0;
    object_found.center.pose.orientation.z = 0.0;
    object_found.center.pose.orientation.w = 1.0;

    // Store the object centroid referenced to the camera frame.
    geometry_msgs::TransformStamped tf_to_cam;
    tf_to_cam = buffer_.lookupTransform(CAMERA_FRAME, "map", ros::Time(0), ros::Duration(1.0));
    tf2::doTransform(object_found.center, object_found.center_cam, tf_to_cam);

    object_found.cluster_original = *cloud;
    // Change point cloud origin to object centroid and save it.
    for (auto &point : cloud->points)
    {
      point.x = point.x - centroid.x;
      point.y = point.y - centroid.y;
      point.z = point.z - centroid.z;
    }
    object_found.cluster = cloud;
    
    // Store mesh.
    if (getMesh) {
      reconstructMesh(cloud, object_found.mesh);
    }
  }

  /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.3, 1.3);
    pass.filter(*cloud);
  }

  /** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
      @param cloud - Pointcloud.
      @param cloud_normals - The point normals once computer will be stored in this. */
  void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
  }

  /** \brief Given the point normals and point indices, extract the normals for the indices.
      @param cloud_normals - Point normals.
      @param inliers - Indices whose normals need to be extracted. */
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                      const pcl::PointIndices::Ptr& inliers)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter(*cloud_normals);
  }

  /** \brief Given the pointcloud remove the biggest plane from the pointcloud. Return True if its the table.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. (output) */
  bool removeBiggestPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_plane, ObjectParams &plane_params)
  {
    // Do Plane Segmentation
    pcl::SACSegmentation<pcl::PointXYZ> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setMaxIterations(1000); /* run at max 1000 iterations before giving up */
    segmentor.setDistanceThreshold(0.01); /* tolerance for variation from model */
    segmentor.setInputCloud(cloud);
    
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    
    /* Extract the planar inliers from the input cloud save them in planeCloud */
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(false);
    extract_indices.filter(*planeCloud);

    /* Extract the planar inliers from the input cloud */
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);

    bool isTheTable = false;

    isTheTable = addPlane(planeCloud, coefficients_plane, plane_params);
    if (isTheTable) {
      pcl::io::savePCDFile("pcl_table.pcd", *planeCloud);
      pcl::io::savePCDFile("pcl_no_table.pcd", *cloud);
    }
    return isTheTable;
  }
  
  /** \brief Calculate distance between two points. */
  float getDistance(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
  }

  /** \brief Bind an object mesh with a 2D detection using distance between two points. */
  void bindDetections(std::vector<ObjectParams> &objects) {
    // Get the best of three different detections.
    boost::shared_ptr<object_detector::objectDetectionArray const> input_detections = ros::topic::waitForMessage<object_detector::objectDetectionArray>("/detections", nh_, ros::Duration(2.5));
    boost::shared_ptr<object_detector::objectDetectionArray const> input_detections2 = ros::topic::waitForMessage<object_detector::objectDetectionArray>("/detections", nh_, ros::Duration(2.5));
    boost::shared_ptr<object_detector::objectDetectionArray const> input_detections3 = ros::topic::waitForMessage<object_detector::objectDetectionArray>("/detections", nh_, ros::Duration(2));
    if (!input_detections) {
      input_detections = input_detections2;
    }
    if (!input_detections) {
      input_detections = input_detections3;
    }
    if (!input_detections) {
      return;
    }
    input_detections = input_detections->detections.size() < input_detections2->detections.size() ? input_detections2 : input_detections;
    input_detections = input_detections->detections.size() < input_detections3->detections.size() ? input_detections3 : input_detections;

    int n_detections = input_detections->detections.size();
    
    for(int i=0;i<n_detections;i++) {
      float min_distance = std::numeric_limits<float>::max();
      int min_index = -1;
      for(int j=0;j<objects.size();j++) {
        float curr_distance = getDistance(objects[j].center_cam.pose.position, input_detections->detections[i].point3D);
        if (curr_distance < min_distance) {
          min_distance = curr_distance;
          min_index = j;
        }
      }
      if (min_index != -1) {
        objects[min_index].label = input_detections->detections[i].label;
        ROS_INFO_STREAM("Detection " << i << " binded with object " << min_index << " -> Min Distance: " << min_distance);
      }
    }

  }

  void findGrasps(const sensor_msgs::PointCloud2& input, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<ObjectParams> &objects) {
    sensor_msgs::PointCloud2 tmp;
    pcl::toROSMsg(objects[0].cluster_original, tmp);
    tmp.header.frame_id = input.header.frame_id;
    tmp.header.stamp = input.header.stamp;;
    pc_pub_1_.publish(tmp);
    pc_pub_2_.publish(input);
    
    // Cloud Sources
    gpd_ros::CloudSources cloud_sources;
    cloud_sources.cloud = input;
    std_msgs::Int64 tmpInt; tmpInt.data = 0;
    cloud_sources.camera_source.assign(cloud->points.size(), tmpInt);

    geometry_msgs::TransformStamped tf_to_cam;
    tf_to_cam = buffer_.lookupTransform("map", CAMERA_FRAME, ros::Time(0), ros::Duration(1.0));
    geometry_msgs::Point tmpPoint;
    tmpPoint.x = tf_to_cam.transform.translation.x;
    tmpPoint.y = tf_to_cam.transform.translation.y;
    tmpPoint.z = tf_to_cam.transform.translation.z;
    cloud_sources.view_points.push_back(tmpPoint);

    gpd_pub_msg_.cloud_sources = cloud_sources;

    ROS_INFO_STREAM("Finding Grasps");
    // Samples
    for(auto &object : objects) {
      gpd_pub_msg_.samples.clear();
      for (auto &point : object.cluster_original.points)
      {
        tmpPoint.x = point.x;
        tmpPoint.y = point.y;
        tmpPoint.z = point.z;
        gpd_pub_msg_.samples.push_back(tmpPoint);
      }
      gpd_pub_.publish(gpd_pub_msg_);
      ROS_INFO_STREAM("Object sended to GPD");
      break; // TO-DO: handle multiple.
    }
  }

  /** \brief PointCloud callback. */
  void cloudCB(const sensor_msgs::PointCloud2& input)
  {
    // Reset Planning Scene Interface
    std::vector<std::string> object_ids = planning_scene_interface_.getKnownObjectNames();
    planning_scene_interface_.removeCollisionObjects(object_ids);
    ros::Duration(2).sleep();
    std_srvs::Empty emptyCall;
    clear_octomap.call(emptyCall);

    // Get cloud ready
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices); // Indices that correspond to a plane.
    pcl::fromROSMsg(input, *cloud);
    pcl::fromROSMsg(input, *cloud_original);
    passThroughFilter(cloud);
    computeNormals(cloud, cloud_normals);

    // Detect and Remove the table on which the object is resting.
    bool tableRemoved = false;
    ObjectParams table_params;
    while(!tableRemoved && !cloud->points.empty()) {
      tableRemoved = removeBiggestPlane(cloud, inliers_plane, table_params);
      extractNormals(cloud_normals, inliers_plane);
    }
    // TO-DO Remove Other Planes.

    if (cloud->points.empty()) {
      return;
    }

    /* Extract all objects from PointCloud using Clustering. */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    getClusters(cloud, clusters);
    int clustersFound = clusters.size();

    std::vector<ObjectParams> objects(clustersFound);
    for(int i=0;i < clustersFound; i++) {
      objects[i].mesh.reset(new shape_msgs::Mesh);
      extractObjectDetails(clusters[i], objects[i]);
    }

    // bindDetections(objects);

    findGrasps(input, cloud_original, objects);

    for(int i=0;i < clustersFound; i++) {
      pcl::io::savePCDFile("pcl_object_"+std::to_string(i)+".pcd", *clusters[i]);
      std::string id = "object-" + std::to_string(i);
      addObject(id, objects[i], table_params, true);
    }
  }

  /** \brief Find all clusters in a pointcloud.*/
  void getClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters) {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    //Set parameters for the clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.025); // 2.5cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(30000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        cloud_cluster->points.push_back(cloud->points[*pit]); //*

      // Discard Noise
      if (cloud_cluster->points.size() < 1000){
        continue; 
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clusters.push_back(cloud_cluster);
    }
  }

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf::TransformListener *tf_listener;
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "detect_objects_and_table");

  // Start the segmentor
  Detect3D segmentor;

  // Spin
  ros::spin();
}
