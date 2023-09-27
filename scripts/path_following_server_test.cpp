#include <algorithm>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/Octomap.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <visualization_msgs/MarkerArray.h>

#define DEPTH 14

// void trajCallback(const nav_msgs::PathConstPtr path_msg)
// {
// }

// void getMap(octomap_msgs::OctomapConstPtr octoMsg)
// {

//     std::shared_ptr<octomap::OcTree> tree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*octoMsg)));

//     std::cout << "getting updated map\n";
//     octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
//     octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
//     std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(abs_tree));

//     // return *tree_obj;
// }

// void timerCallback(const ros::TimerEvent &)
// {
//     octomap_msgs::OctomapConstPtr octoMsg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
//     std::shared_ptr<octomap::OcTree> tree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*octoMsg)));

//     octomap::point3d ptA(0, 0, 0);
//     octomap::point3d ptB(0, 5, 3);
//     std::vector<octomap::point3d> nodes;
//     tree->computeRay(ptA, ptB, nodes);

//     for (octomap::point3d elem : nodes)
//     {
//         std::cout << elem << "\n";
//     }

//     // octomap::OcTreeNode *dummyNode = tree->search(pt, DEPTH);
// }

int main(int argc, char **argv)
{
    std::cout << "started path following server\n";
    ros::init(argc, argv, "path_following_server");
    ros::NodeHandle nh("~");

    // ros::Subscriber sub = nh.subscribe("path_following_server/path_request", 10, trajCallback);

    // ros::Subscriber sub = nh.subscribe("/octomap_binary", 10, getMap);

    // ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

    // crear el marker
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/rayhits", 10);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world_ned";
    marker.ns = "hits";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.659;
    marker.color.g = 0.659;
    marker.color.b = 0.659;
    marker.color.a = 1;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world_ned", "/rviz_visual_markers"));

    while (ros::ok())
    {
        marker.header.stamp = ros::Time::now();
        marker.points.clear();
        visual_tools_->deleteAllMarkers();

        // octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
        // octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
        // std::shared_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree *>(abs_tree));

        octomap_msgs::OctomapConstPtr msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
        std::shared_ptr<octomap::OcTree> tree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));

        octomap::point3d ptA(1, 1, 1);
        octomap::point3d ptB(0, 5, 3);

        octomap::point3d hit;
        tree->castRay(ptA, ptB - ptA, hit, true);

        std::cout << "ray hit at: " << hit << "\n";

        geometry_msgs::Point pt_msgs;
        pt_msgs.x = hit.x();
        pt_msgs.y = hit.y();
        pt_msgs.z = hit.z();
        marker.points.push_back(pt_msgs);

        marker_pub.publish(marker);

        Eigen::Vector3d start = {ptA.x(), ptA.y(), ptA.z()};
        Eigen::Vector3d end = {ptB.x(), ptB.y(), ptB.z()};

        visual_tools_->publishLine(start, end);

        // Don't forget to trigger the publisher!
        visual_tools_->trigger();

        ros::spinOnce();
        ros::Duration(0.1).sleep();

        // ros::spin();
    }
}