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

// FCL STUFF
// #include <fcl/fcl.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

#define DEPTH 14

rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
int counter = 0;

class CollisionManager
{
public:
    nav_msgs::PathConstPtr path_;

    // auv collision object
    std::shared_ptr<fcl::Boxf> auv_box_;
    std::shared_ptr<fcl::CollisionObjectf> auv_co_;
    std::shared_ptr<fcl::CollisionObjectf> tree_obj_;
    ros::Timer timer_;
    std::string valor = "";

    fcl::CollisionResultf col_res_;

    CollisionManager(ros::NodeHandle nh)
    {
        auv_box_ = std::shared_ptr<fcl::Boxf>(new fcl::Boxf(1.7, 1.2, 1.3));
        auv_co_ = std::shared_ptr<fcl::CollisionObjectf>(new fcl::CollisionObjectf(auv_box_));

        timer_ = nh.createTimer(ros::Duration(0.1), &CollisionManager::collisionChecker, this, false, false);
        // col_res_.clear();
    }

    void getMap(octomap_msgs::OctomapConstPtr octoMsg)
    {
        std::cout << "new map gotten\n";
        std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*octoMsg)));

        // convertirlo a collision object
        std::shared_ptr<fcl::OcTreef> tree(new fcl::OcTreef(octree));
        std::shared_ptr<fcl::CollisionGeometryf> geo(tree);
        tree_obj_ = std::shared_ptr<fcl::CollisionObjectf>(new fcl::CollisionObjectf(geo));
        // return *tree_obj;
    }

    void collisionChecker(const ros::TimerEvent &)
    {
        std::cout << "collision checking started\n";

        std::cout << "size of path in colMan: " << path_->poses.size() << "\n";
        std::cout << "counter: " << counter << "\n";

        if (tree_obj_ == NULL)
        {
            std::cout << "TREE IS VOID\n";
            return;
        }

        // path_->poses.at(0).pose.position
        for (auto pose = path_->poses.begin() + counter; pose != path_->poses.end(); ++pose)
        {
            std::cout << "for ok:" << pose->pose.position.x << " \n";

            // auv position to compute collision
            auv_co_->setTranslation(Eigen::Vector3f(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
            auv_co_->setRotation(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()).matrix());
            std::cout << "auv collision ok:" << pose->pose.position.y << " \n";

            // do collision request
            fcl::CollisionRequestf col_req_;
            // fcl::CollisionResultf col_res_;
            fcl::collide(tree_obj_.get(), auv_co_.get(), col_req_, col_res_);

            if (col_res_.isCollision())
            {
                std::cout << "colision! \n";
            }
            else
            {
                std::cout << "its free! \n";
            }
        }
    }
};

class PathManager
{
public:
    nav_msgs::PathConstPtr path_;

    std::shared_ptr<CollisionManager> colMan_;
    // CollisionManager colMan_;

    ros::Timer timer;

    PathManager(ros::NodeHandle nh, CollisionManager &colman) : colMan_(&colman)
    {
        std::cout << "el collision manager object es: " << colMan_->valor << "\n";
        timer = nh.createTimer(ros::Duration(1), &PathManager::manager, this, false, false);
    }
    void trajCallback(const nav_msgs::PathConstPtr pathMsg)
    {
        std::cout << "path gotten\n";
        path_ = pathMsg;
        std::cout << "size of path in pathMan: " << path_->poses.size() << "\n";
        colMan_->path_ = pathMsg;
        std::vector<geometry_msgs::Pose> path;

        std::vector<rviz_visual_tools::colors> colorlist_ = {rviz_visual_tools::WHITE, rviz_visual_tools::BLUE};
        std::vector<rviz_visual_tools::colors> colors;
        int i = 0;
        for (geometry_msgs::PoseStamped elem : pathMsg->poses)
        {
            path.push_back(elem.pose);
            colors.push_back(colorlist_.at(i++ % colorlist_.size()));
            visual_tools_->publishArrow(elem.pose, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
        }

        visual_tools_->publishPath(path, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
        visual_tools_->trigger();

        timer.start();
        colMan_->timer_.start();
    }

    void manager(const ros::TimerEvent &)
    {
        // pop from path
        std::cout << "size of path in pathMan manager(): " << path_->poses.size() << "\n";
        path_->poses.at(0);
        if (colMan_->col_res_.isCollision())
        {
            std::cout << "STOPed manager: \n";
            timer.stop();
            colMan_->timer_.stop();

            if (counter < path_->poses.size())
            {
                counter++;
            }
            else
            {
                std::cout << "STOPed manager: \n";
                timer.stop();
                colMan_->timer_.stop();
            }
        }
        else
        {
            std::cout << "STOPed manager: \n";
            timer.stop();
            colMan_->timer_.stop();
        }
    }
};

void timerCallback(const ros::TimerEvent &)
{
}

nav_msgs::PathConstPtr pathMsg_;

int main(int argc, char **argv)
{
    std::cout << "started path following server\n";
    ros::init(argc, argv, "path_following_server");
    ros::NodeHandle nh;

    CollisionManager colMan(nh);
    colMan.valor = "jueves";
    ros::Subscriber sub1 = nh.subscribe("/octomap_binary", 10, &CollisionManager::getMap, &colMan);

    PathManager pathMan(nh, colMan);
    ros::Subscriber sub = nh.subscribe("planner/path_result", 10, &PathManager::trajCallback, &pathMan);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

    // For visualizing things in rviz
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world_ned", "/rviz_visual_markers"));

    while (ros::ok())
    {

        // visual_tools_->deleteAllMarkers();

        // get el octomap
        // octomap_msgs::OctomapConstPtr msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
        // std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
        // // convertirlo a collision object
        // std::shared_ptr<fcl::OcTreef> tree(new fcl::OcTreef(octree));
        // std::shared_ptr<fcl::CollisionGeometryf> geo(tree);
        // std::shared_ptr<fcl::CollisionObjectf> tree_obj(new fcl::CollisionObjectf(geo));
        // // auv collision object
        // std::shared_ptr<fcl::Boxf> auv_box_;
        // std::shared_ptr<fcl::CollisionObjectf> auv_co_;
        // auv_box_ = std::shared_ptr<fcl::Boxf>(new fcl::Boxf(1.7, 1.2, 1.3));
        // auv_co_.reset(new fcl::CollisionObjectf(auv_box_));
        // // auv position to compute collision
        // auv_co_->setTranslation(Eigen::Vector3f(0, 4, 3));
        // auv_co_->setRotation(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()).matrix());
        // // do collision request
        // fcl::CollisionRequestf col_req_;
        // fcl::CollisionResultf col_res_;
        // fcl::collide(tree_obj.get(), auv_co_.get(), col_req_, col_res_);

        // if (col_res_.isCollision())
        // {
        //     std::cout << "colision! \n";
        //     // std::cout << auv_co_->getTranslation() << "\n";
        //     // return false;
        // }

        // Eigen::Vector3d start = {0, 0, 0};
        // Eigen::Vector3d end = {0, 5, 3};

        // visual_tools_->publishLine(start, end);

        // Don't forget to trigger the publisher!
        // visual_tools_->trigger();

        ros::spinOnce();
        ros::Duration(0.1).sleep();

        // ros::spin();
    }
}