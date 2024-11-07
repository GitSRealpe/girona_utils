#include <algorithm>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <cola2_msgs/BodyVelocityReq.h>
#include <geometry_msgs/TwistStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib/server/simple_action_server.h>
#include <girona_utils/PursuitAction.h>

#include <functional> // For std::function

#define MAX_SPEED 0.7
#define MAX_ROTATION_SPEED 0.35

class PursuitController
{

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<girona_utils::PursuitAction> as_;
    std::string action_name_;
    girona_utils::PursuitFeedback feedback_;
    girona_utils::PursuitResult result_;
    ros::Timer timer_;

public:
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf2_ros::Buffer tfBuffer;
    geometry_msgs::TransformStamped t;

    geometry_msgs::Pose setPoint;
    tf2::Transform mat_curr;
    tf2::Transform mat_goal;
    tf2::Transform error;
    double roll, pitch, err_yaw, integral, derivative, last_error_x, last_error_y, last_error_z = 0;
    ros::Duration dt;
    ros::Time prev_t;
    Eigen::Matrix3d pid_err;

    nav_msgs::Path path;

    ros::Publisher pubCOLA2;
    cola2_msgs::BodyVelocityReq vel_req;

    double max_vel = MAX_SPEED;
    double max_rot_vel = MAX_ROTATION_SPEED;

    std::string interface = "nothing";
    std::string base_link;
    std::string velocity_topic;

    visualization_msgs::Marker sphere_m;
    visualization_msgs::Marker intersection1_m;
    visualization_msgs::Marker next_m;
    visualization_msgs::MarkerArray markers;
    ros::Publisher pubviz;

    Eigen::Array3d p0, p1, sph;
    double radius;
    int waypoint_index;

    PursuitController(std::string name) : as_(nh_, name, false), action_name_(name)
    {

        ros::NodeHandle nhp("~");
        nhp.getParam("max_vel", max_vel);
        nhp.getParam("max_rot_vel", max_rot_vel);
        nhp.getParam("output_interface", interface);
        nhp.getParam("base_link", base_link);
        nhp.getParam("velocity_topic", velocity_topic);
        std::cout << "using " << interface << " as velocity controller \n";
        std::cout << "using " << base_link << " as base_link \n";
        std::cout << "using " << velocity_topic << " as velocity topic \n";
        std::cout << "max velocity set at " << max_vel << " m/s \n";
        std::cout << "max rotation velocity set at " << max_rot_vel << " m/s \n";

        sphere_m.type = visualization_msgs::Marker::SPHERE;
        sphere_m.id = 0;
        sphere_m.color.a = 0.7;
        sphere_m.color.r = 1;
        sphere_m.color.g = 1;
        sphere_m.color.b = 1;
        //  poner esto en el frame del girona asi no tener que hacer update con la odometria?
        sphere_m.header.frame_id = "world_ned";
        sphere_m.pose.orientation.w = 1;

        intersection1_m.type = visualization_msgs::Marker::SPHERE;
        intersection1_m.header.frame_id = "world_ned";
        intersection1_m.id = 1;
        intersection1_m.color.a = 1;
        intersection1_m.color.g = 1;
        intersection1_m.scale.x = 0.1;
        intersection1_m.scale.y = 0.1;
        intersection1_m.scale.z = 0.1;
        intersection1_m.pose.orientation.w = 1;

        next_m = intersection1_m;
        next_m.id = 2;
        markers.markers.push_back(sphere_m);
        markers.markers.push_back(intersection1_m);
        markers.markers.push_back(next_m);

        pubviz = nh_.advertise<visualization_msgs::MarkerArray>("pursuit_viz", 5, false);

        // register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&PursuitController::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&PursuitController::preemptCB, this));

        std::cout << "Initializing PursuitController controller\n";
        tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
        while (ros::ok())
        {
            try
            {
                t = tfBuffer.lookupTransform("world_ned", base_link, ros::Time(0));
                std::cout << "transform gotten\n";
                std::cout << t.transform.translation << "\n";
                break;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(0.5).sleep();
                continue;
            }
        }

        pid_err.setZero();
        // prev_t = ros::Time::now();
        prev_t = ros::Time(0);

        pubCOLA2 = nh_.advertise<cola2_msgs::BodyVelocityReq>(velocity_topic, 5, false);
        timer_ = nh_.createTimer(ros::Rate(10), &PursuitController::update, this, false, false);
        as_.start();
    }

    void sendVelCOLA2(Eigen::Matrix3d err, double yaw)
    {
        vel_req.header.frame_id = "world_ned";
        vel_req.goal.requester = "sebas";
        vel_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;
        // vel_req.disable_axis.x = false;
        // vel_req.disable_axis.y = false;
        // vel_req.disable_axis.z = false;
        // vel_req.disable_axis.yaw = false;
        vel_req.disable_axis.roll = true;
        vel_req.disable_axis.pitch = true;
        vel_req.twist.linear.x = std::clamp(err.row(0).sum(), -max_vel, max_vel);
        vel_req.twist.linear.y = std::clamp(err.row(1).sum(), -max_vel, max_vel);
        vel_req.twist.linear.z = std::clamp(err.row(2).sum(), -max_vel, max_vel);
        vel_req.twist.angular.z = std::clamp(0.7 * yaw, -max_rot_vel, max_rot_vel);
        vel_req.header.stamp = ros::Time::now();
        pubCOLA2.publish(vel_req);
    }

    void changeVel(const std_msgs::Float32ConstPtr msg)
    {
        std::cout << "vel changed to " << msg->data << "\n";
        max_vel = msg->data;
    }

    void goalCB()
    {
        // reset helper variables
        // KLSDALSDLASDKJL
        // accept the new goal
        timer_.stop();
        auto goal = as_.acceptNewGoal();
        path = goal->path;
        radius = goal->radius;

        sphere_m.scale.x = radius * 2;
        sphere_m.scale.y = radius * 2;
        sphere_m.scale.z = radius * 2;

        waypoint_index = 1;

        // as_.isPreemptRequested(); ?
        timer_.start();
    }

    void preemptCB()
    {

        // set the action state to preempted

        if (as_.isNewGoalAvailable())
        {
            ROS_INFO("%s: Following new goal", action_name_.c_str());
        }
        else
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            // as_.shutdown(); ?
            timer_.stop();
        }
    }

    std::vector<Eigen::Array3d> compute_intersection()
    {
        std::vector<Eigen::Array3d> intersections;

        t = tfBuffer.lookupTransform("world_ned", base_link, ros::Time(0));
        sph[0] = t.transform.translation.x;
        sph[1] = t.transform.translation.y;
        sph[2] = t.transform.translation.z;

        sphere_m.pose.position.x = sph[0];
        sphere_m.pose.position.y = sph[1];
        sphere_m.pose.position.z = sph[2];
        markers.markers.at(0) = sphere_m;

        p0 = {path.poses[waypoint_index - 1].pose.position.x,
              path.poses[waypoint_index - 1].pose.position.y,
              path.poses[waypoint_index - 1].pose.position.z};
        p1 = {path.poses[waypoint_index].pose.position.x,
              path.poses[waypoint_index].pose.position.y,
              path.poses[waypoint_index].pose.position.z};
        // sometimes p0 and p1 are the same point, with different orientation
        if ((p0 - p1).pow(2).sum() < DBL_EPSILON)
        {
            waypoint_index++;
            return intersections;
        }

        double dx = p1[0] - p0[0];
        double dy = p1[1] - p0[1];
        double dz = p1[2] - p0[2];

        double a = pow((dx), 2) + pow((dy), 2) + pow((dz), 2);
        double b = 2 * (dx * (p0[0] - sph[0]) + dy * (p0[1] - sph[1]) + dz * (p0[2] - sph[2]));
        double c = pow((p0[0] - sph[0]), 2) + pow((p0[1] - sph[1]), 2) + pow((p0[2] - sph[2]), 2) - pow(radius, 2);

        double discriminant = pow(b, 2) - (4 * a * c);
        if (discriminant < 0)
        {
            // no intersection
            return intersections;
        }
        else
        {
            if (std::abs(discriminant) < DBL_EPSILON)
            {
                // one intersection
                double t = -b / (2 * a);
                std::cout << t << "\n";
                Eigen::Array3d intersection = p0 + (t * p1);
                std::cout << intersection << "\n";

                intersections.push_back(intersection);
                return intersections;
            }
            else
            {
                // two intersections
                double t1 = (-b + sqrt(discriminant)) / (2 * a);
                double t2 = (-b - sqrt(discriminant)) / (2 * a);
                Eigen::Array3d intersection1 = (p0) * (1 - t1) + (p1 * t1);
                Eigen::Array3d intersection2 = (p0) * (1 - t2) + (p1 * t2);

                intersections.push_back(intersection1);
                intersections.push_back(intersection2);
                return intersections;
            }
        }

        return intersections;
    }

    void update(const ros::TimerEvent &event)
    {

        std::vector<Eigen::Array3d> intersections = compute_intersection();

        double dist1;
        double dist2;
        int i;

        switch (intersections.size())
        {
        case 0:
            std::cout << "no intersections\n";
            // last lead point or waypoint in path
            setPoint = path.poses[waypoint_index].pose;
            break;
        case 1:
            std::cout << "one intersection\n";
            setPoint.position.x = intersections[0][0];
            setPoint.position.y = intersections[0][1];
            setPoint.position.z = intersections[0][2];

            dist1 = ((p1 - intersections[0]).pow(2)).sum();
            if (dist1 < 0.01)
            {
                // if distance is less than 0.01 then get next point in path
                waypoint_index++;
                std ::cout << "index" << waypoint_index << "\n";
            }

            break;
        case 2:
            std::cout << "two intersections\n";
            dist1 = ((p1 - intersections[0]).pow(2)).sum();
            dist2 = ((p1 - intersections[1]).pow(2)).sum();
            // the lead point is the one close to the next goal
            i = (dist1 < dist2) ? 0 : 1;

            setPoint.position.x = intersections[i][0];
            setPoint.position.y = intersections[i][1];
            setPoint.position.z = intersections[i][2];

            if ((sph - p1).pow(2).sum() < (intersections[i] - p1).pow(2).sum())
            {
                // if point is inside the sphere advance to next one
                waypoint_index++;
                return;
            }

            if (dist1 < 0.01 || dist2 < 0.01)
            {
                // if distance is less than 0.01 then get next point in path
                waypoint_index++;
                std ::cout << "index" << waypoint_index << "\n";
            }
            break;

        default:
            break;
        }

        if (waypoint_index < path.poses.size())
        {
            std::cout << "last waypoint reached" << "\n";
            markers.markers.at(1).pose = setPoint;
            markers.markers.at(2).pose = path.poses[waypoint_index].pose;
            setPoint.orientation = path.poses[waypoint_index].pose.orientation;
        }
        else
        {
            setPoint = path.poses.back().pose;
        }

        pubviz.publish(markers);

        feedback_.waypoint = waypoint_index;
        feedback_.setPoint = setPoint;
        as_.publishFeedback(feedback_);

        tf2::fromMsg(t.transform, mat_curr);
        tf2::fromMsg(setPoint, mat_goal);
        error = mat_curr.inverseTimes(mat_goal);

        // get orientation as rpy
        tf2::Matrix3x3 m(error.getRotation());
        m.getRPY(roll, pitch, err_yaw, 1);

        // proportional
        pid_err(0, 0) = 1 * error.getOrigin().x();
        pid_err(1, 0) = 1 * error.getOrigin().y();
        pid_err(2, 0) = 1 * error.getOrigin().z();

        // integral for steady state error, but adds inestability
        dt = ros::Time::now() - prev_t;
        // integral += error.getOrigin().z() * dt.toSec();
        // pid_err(2, 1) = 0.1 * integral;

        // derivative smooths
        derivative = (error.getOrigin().x() - last_error_x) / dt.toSec();
        pid_err(0, 2) = 0.5 * derivative;
        derivative = (error.getOrigin().y() - last_error_y) / dt.toSec();
        pid_err(1, 2) = 0.5 * derivative;
        derivative = (error.getOrigin().z() - last_error_z) / dt.toSec();
        pid_err(2, 2) = 0.5 * derivative;

        last_error_x = error.getOrigin().x();
        last_error_y = error.getOrigin().y();
        last_error_z = error.getOrigin().z();

        std::cout << "x action:" << std::clamp(pid_err.row(0).sum(), -max_vel, max_vel) << "\n";
        std::cout << "y action:" << std::clamp(pid_err.row(1).sum(), -max_vel, max_vel) << "\n";
        std::cout << "z action:" << std::clamp(pid_err.row(2).sum(), -max_vel, max_vel) << "\n";
        std::cout << "yaw action:" << std::clamp(0.7 * err_yaw, -max_rot_vel, max_rot_vel) << "\n";
        // std::cout << "raw_yaw: " << err_yaw << "\n";

        sendVelCOLA2(pid_err, err_yaw);

        prev_t = ros::Time::now();
    }
};

int main(int argc, char **argv)
{
    std::cout << "executing path\n";
    ros::init(argc, argv, "pursuit_controller");
    ros::NodeHandle n;

    PursuitController pursuitc(ros::this_node::getName());

    ros::Subscriber sub = n.subscribe(ros::this_node::getName() + "/vel_target", 10, &PursuitController::changeVel, &pursuitc);

    ros::spin();
}