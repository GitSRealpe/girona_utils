// SRealpe
#include <algorithm>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

// ros msg stuff
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>
#include <girona_utils/PathStatus.h>

// PID action msgs
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <girona_utils/PIDAction.h>

// octomap stuff
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/Octomap.h>

// rviz visual tools
#include <rviz_visual_tools/rviz_visual_tools.h>

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
    // collision objects
    std::shared_ptr<fcl::Boxf> auv_box_;
    std::shared_ptr<fcl::CollisionObjectf> auv_co_;
    std::shared_ptr<fcl::CollisionObjectf> tree_obj_;
    fcl::CollisionResultf col_res_;
    Eigen::Isometry3d poseMat;
    // Eigen::Affine3f poseMat;

    // ros
    ros::Subscriber sub;
    ros::Timer timer_;
    nav_msgs::PathConstPtr path_;
    visualization_msgs::Marker marker;

    std::string valor = "";

    CollisionManager(ros::NodeHandle nh)
    {
        sub = nh.subscribe("/octomap_binary", 10, &CollisionManager::getMap, this);

        auv_box_ = std::shared_ptr<fcl::Boxf>(new fcl::Boxf(1.7, 1.2, 1.3));
        auv_co_ = std::shared_ptr<fcl::CollisionObjectf>(new fcl::CollisionObjectf(auv_box_));

        timer_ = nh.createTimer(ros::Duration(1), &CollisionManager::collisionChecker, this, false, false);

        // crear el marker
        marker.header.frame_id = "world_ned";
        marker.ns = "auv_box";
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.4;
        marker.pose.orientation.w = 1;
        marker.scale.x = 1.7;
        marker.scale.y = 1.2;
        marker.scale.z = 1.3;
    }

    void getMap(octomap_msgs::OctomapConstPtr octoMsg)
    {
        // std::cout << "new map gotten\n";
        std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*octoMsg)));
        // convertirlo a collision object
        std::shared_ptr<fcl::OcTreef> tree(new fcl::OcTreef(octree));
        std::shared_ptr<fcl::CollisionGeometryf> geo(tree);
        tree_obj_ = std::shared_ptr<fcl::CollisionObjectf>(new fcl::CollisionObjectf(geo));
    }

    void collisionChecker(const ros::TimerEvent &)
    {
        std::cout << "checking collision from state: " << counter << " of: " << path_->poses.size() << "\n";

        if (tree_obj_ == NULL)
        {
            std::cout << "TREE IS VOID\n";
            return;
        }

        for (auto poseS = path_->poses.begin() + counter; poseS != path_->poses.end(); ++poseS)
        {
            // auv position to compute collision
            auv_co_->setTranslation(Eigen::Vector3f(poseS->pose.position.x, poseS->pose.position.y, poseS->pose.position.z));
            tf2::fromMsg(poseS->pose, poseMat);

            auv_co_->setRotation(poseMat.rotation().cast<float>());

            // std::cout << "auv collision ok:" << poseS->pose.position.y << " \n";

            // do collision request
            fcl::CollisionRequestf col_req_;
            // fcl::CollisionResultf col_res_;
            fcl::collide(tree_obj_.get(), auv_co_.get(), col_req_, col_res_);
            // fcl::collide(&*tree_obj_, auv_co_.get(), col_req_, col_res_);

            if (col_res_.isCollision())
            {
                std::cout << "colision! \n";
                ROS_WARN("Detected collision in path.");
                timer_.stop();
                marker.pose = poseS->pose;
                visual_tools_->publishMarker(marker);
                visual_tools_->trigger();
                break;
            }
        }
    }
};

class PathManager
{
public:
    nav_msgs::PathConstPtr path_;

    std::shared_ptr<CollisionManager> colMan_;

    ros::Timer manTimer;
    ros::Subscriber subFeed;
    ros::Publisher pathStPub;
    girona_utils::PathStatus pathStatus;

    std::shared_ptr<actionlib::SimpleActionClient<girona_utils::PIDAction>> pidClient;
    Eigen::Isometry3d feedMat;
    double error = 0;

    PathManager(ros::NodeHandle nh)
    {
        // manTimer = nh.createTimer(ros::Duration(0.5), &PathManager::manager, this, false, false);
        colMan_ = std::make_shared<CollisionManager>(nh);
        colMan_->valor = "lunes";
        std::cout << "el collision manager object es: " << colMan_->valor << "\n";

        pathStPub = nh.advertise<girona_utils::PathStatus>("path_manager_server/status", 1, true);
        pathStatus.status = girona_utils::PathStatus::NO_PATH;
        pathStPub.publish(pathStatus);

        pidClient = std::make_shared<actionlib::SimpleActionClient<girona_utils::PIDAction>>("pid_controller");
        ROS_INFO("Waiting for PID action server to start.");
        pidClient->waitForServer(); // will wait for infinite time
        ROS_INFO("PID Action server is active, waiting for path.");
        subFeed = nh.subscribe("/pid_controller/feedback", 10, &PathManager::feedbackCb, this);
    }

    void trajCallback(const nav_msgs::PathConstPtr pathMsg)
    {
        std::cout << "path gotten\n";
        pathStatus.status = girona_utils::PathStatus::PATH_RECEIVED;
        pathStPub.publish(pathStatus);
        path_ = pathMsg;
        std::cout << "size of path in pathMan: " << path_->poses.size() << "\n";
        colMan_->path_ = pathMsg;

        counter = 0;
        // std::vector<geometry_msgs::Pose> path;
        // for (geometry_msgs::PoseStamped elem : pathMsg->poses)
        // {
        //     path.push_back(elem.pose);
        //     visual_tools_->publishArrow(elem.pose, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
        // }
        // visual_tools_->publishPath(path, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
        // visual_tools_->trigger();

        colMan_->col_res_.clear();
        // colMan_->timer_.start();
        ROS_INFO("Path received, waiting for trigger signal.");
    }

    bool trigger(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
    {
        ROS_INFO("Trigger command received, following path.");
        pathStatus.status = girona_utils::PathStatus::STARTED;
        pathStPub.publish(pathStatus);
        // manTimer.start();
        colMan_->timer_.start();
        manager();
        return true;
    }

    // Called every time feedback is received for the goal
    void feedbackCb(const girona_utils::PIDActionFeedbackConstPtr &feedMsg)
    {
        feedMsg->feedback.current;
        tf2::fromMsg(feedMsg->feedback.current, feedMat);
        feedMat.translation().norm();
        // std::cout << "error: " << feedMat.translation().norm() << "\n";
    }

    void manager()
    {
        while (true)
        {
            if (colMan_->col_res_.isCollision())
            {
                std::cout << "Stoped path following \n";
                pathStatus.status = girona_utils::PathStatus::STOPPED;
                pathStPub.publish(pathStatus);
                colMan_->timer_.stop();
                return;
            }
            else
            {
                // pop from path
                if (feedMat.translation().norm() < 0.2)
                {
                    if (counter < path_->poses.size())
                    {
                        pathStatus.status = girona_utils::PathStatus::RUNNING;
                        pathStPub.publish(pathStatus);
                        girona_utils::PIDGoal goal;
                        goal.goal = path_->poses.at(counter).pose;
                        pidClient->sendGoal(goal);
                        counter++;
                    }
                    else
                    {
                        std::cout << "end reached\n";
                        pathStatus.status = girona_utils::PathStatus::END_REACHED;
                        pathStPub.publish(pathStatus);
                        colMan_->timer_.stop();
                        return;
                    }
                }
            }
            // ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
    }

    // void manager(const ros::TimerEvent &)
    // {
    //     if (colMan_->col_res_.isCollision())
    //     {
    //         std::cout << "Stoped path manager \n";
    //         manTimer.stop();
    //     }
    //     else
    //     {
    //         // pop from path
    //         if (feedMat.translation().norm() < 0.2)
    //         {
    //             if (counter < path_->poses.size())
    //             {
    //                 girona_utils::PIDGoal goal;
    //                 goal.goal = path_->poses.at(counter).pose;
    //                 pidClient->sendGoal(goal);

    //                 counter++;
    //             }
    //             else
    //             {
    //                 std::cout << "end reached\n";
    //                 manTimer.stop();
    //             }
    //         }
    //     }
    // }
};

int main(int argc, char **argv)
{
    std::cout << "started path following server\n";
    ros::init(argc, argv, "path_manager_server");
    ros::NodeHandle nh;

    PathManager pathMan(nh);
    ros::Subscriber sub = nh.subscribe("planner/path_result", 10, &PathManager::trajCallback, &pathMan);
    ros::ServiceServer service = nh.advertiseService("path_manager_server/startPath", &PathManager::trigger, &pathMan);

    // For visualizing things in rviz
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world_ned", "/rviz_visual_markers"));

    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();

    //     // ros::spin();
    // }
}