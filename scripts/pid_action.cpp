#include <algorithm>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <cola2_msgs/BodyVelocityReq.h>

#include <actionlib/server/simple_action_server.h>
#include <girona_utils/PIDAction.h>

#define MAX_SPEED 0.7

class PID
{

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<girona_utils::PIDAction> as_;
    std::string action_name_;
    girona_utils::PIDFeedback feedback_;
    girona_utils::PIDResult result_;
    ros::Timer timer_;

public:
    ros::Publisher pub;
    geometry_msgs::TransformStamped t;
    geometry_msgs::Pose setPoint;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf2_ros::Buffer tfBuffer;
    tf2::Transform mat_curr;
    tf2::Transform mat_goal;
    tf2::Transform error;
    double roll, pitch, err_yaw, integral, derivative, last_error_x, last_error_y, last_error_z = 0;
    ros::Duration dt;
    ros::Time prev_t;
    Eigen::Matrix3d pid_err;
    bool near = false;
    cola2_msgs::BodyVelocityReq vel_req;

    double max_vel = MAX_SPEED;

    PID(std::string name) : as_(nh_, name, false), action_name_(name)
    {
        // register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&PID::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&PID::preemptCB, this));

        std::cout << "Initializing PID controller\n";
        tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
        while (ros::ok())
        {
            try
            {
                t = tfBuffer.lookupTransform("world_ned", "girona1000/base_link", ros::Time(0));
                break;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(0.5).sleep();
                continue;
            }
        }
        prev_t = ros::Time::now();

        vel_req.header.frame_id = "world_ned";
        vel_req.goal.requester = "sebas";
        vel_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;
        pub = nh_.advertise<cola2_msgs::BodyVelocityReq>("/girona1000/controller/body_velocity_req", 5, true);

        timer_ = nh_.createTimer(ros::Rate(10), &PID::update, this, false, false);
        as_.start();
    };

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
        setPoint = as_.acceptNewGoal()->goal;
        as_.isPreemptRequested();
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
            timer_.stop();
        }
    }

    void update(const ros::TimerEvent &event)
    {

        // std::cout << "controlando\n";

        tf2::fromMsg(setPoint, mat_goal);
        t = tfBuffer.lookupTransform("world_ned", "girona1000/base_link", ros::Time(0));
        tf2::fromMsg(t.transform, mat_curr);
        error = mat_curr.inverseTimes(mat_goal);

        tf2::toMsg(error, feedback_.current);
        as_.publishFeedback(feedback_);

        // get orientation as rpy
        tf2::Matrix3x3 m(error.getRotation());
        m.getRPY(roll, pitch, err_yaw, 1);

        // proportional
        pid_err(0, 0) = 1 * error.getOrigin().x();
        pid_err(1, 0) = 1 * error.getOrigin().y();
        pid_err(2, 0) = 1 * error.getOrigin().z();

        // integral for steady state error, but adds inestability
        dt = ros::Time::now() - prev_t;
        // integral += error.getOrigin().y() * dt.toSec();
        // pid_err(1, 1) = 0.1 * integral;

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

        prev_t = ros::Time::now();

        std::cout << "x action:" << std::clamp(pid_err.row(0).sum(), -max_vel, max_vel) << "\n";
        std::cout << "y action:" << std::clamp(pid_err.row(1).sum(), -max_vel, max_vel) << "\n";
        std::cout << "z action:" << std::clamp(pid_err.row(2).sum(), -max_vel, max_vel) << "\n";
        std::cout << "yaw action:" << std::clamp(0.7 * err_yaw, -0.785, 0.785) << "\n";
        // std::cout << "raw_yaw: " << err_yaw << "\n";
        vel_req.twist.linear.x = std::clamp(pid_err.row(0).sum(), -max_vel, max_vel);
        vel_req.twist.linear.y = std::clamp(pid_err.row(1).sum(), -max_vel, max_vel);
        vel_req.twist.linear.z = std::clamp(pid_err.row(2).sum(), -max_vel, max_vel);
        vel_req.twist.angular.z = std::clamp(0.7 * err_yaw, -0.785, 0.785);
        vel_req.header.stamp = ros::Time::now();

        pub.publish(vel_req);

        // near = error.getOrigin().length() < 0.2;
    }
};

int main(int argc, char **argv)
{
    std::cout << "executing path\n";
    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    PID pid(ros::this_node::getName());
    nh.getParam("max_vel", pid.max_vel);

    ros::Subscriber sub = n.subscribe(ros::this_node::getName() + "/vel_target", 10, &PID::changeVel, &pid);

    ros::spin();
}
