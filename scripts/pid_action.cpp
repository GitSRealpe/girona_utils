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

#include <actionlib/server/simple_action_server.h>
#include <girona_utils/PIDAction.h>

#include <functional> // For std::function

#define MAX_SPEED 0.7
#define MAX_ROTATION_SPEED 0.35

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

    ros::Publisher pubCOLA2;
    ros::Publisher pubTPvel;
    ros::Publisher pubTPdummy;
    cola2_msgs::BodyVelocityReq vel_req;
    geometry_msgs::TwistStamped vel_tp_req;

    double max_vel = MAX_SPEED;
    double max_rot_vel = MAX_ROTATION_SPEED;

    std::string interface = "nothing";

    PID(std::string name) : as_(nh_, name, false), action_name_(name)
    {

        ros::NodeHandle nhp("~");
        nhp.getParam("max_vel", max_vel);
        nhp.getParam("max_rot_vel", max_rot_vel);
        nhp.getParam("output_interface", interface);
        std::cout << "using " << interface << " as velocity controller \n";
        std::cout << "max velocity set at " << max_vel << " m/s \n";
        std::cout << "max rotation velocity set at " << max_rot_vel << " m/s \n";

        // register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&PID::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&PID::preemptCB, this));

        std::cout << "Initializing PID controller\n";
        tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
        while (ros::ok())
        {
            try
            {
                t = tfBuffer.lookupTransform("world_ned", "girona500/origin", ros::Time(0));
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

        pubCOLA2 = nh_.advertise<cola2_msgs::BodyVelocityReq>("/girona500/controller/body_velocity_req", 5, false);
        pubTPvel = nh_.advertise<geometry_msgs::TwistStamped>("/girona500/tp_controller/tasks/auv_configuration/feedforward", 5, false);

        pubTPdummy = nh_.advertise<geometry_msgs::PoseStamped>("/girona500/tp_controller/tasks/auv_configuration/target", 5, false);

        timer_ = nh_.createTimer(ros::Rate(10), &PID::update, this, false, false);
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

    void sendVelTP(Eigen::Matrix3d err, double yaw)
    {
        vel_tp_req.header.frame_id = "world_ned";
        vel_tp_req.header.stamp = ros::Time::now();
        vel_tp_req.twist.linear.x = std::clamp(err.row(0).sum(), -max_vel, max_vel);
        vel_tp_req.twist.linear.y = std::clamp(err.row(1).sum(), -max_vel, max_vel);
        vel_tp_req.twist.linear.z = std::clamp(err.row(2).sum(), -max_vel, max_vel);
        vel_tp_req.twist.angular.z = std::clamp(0.7 * yaw, -max_rot_vel, max_rot_vel);
        pubTPvel.publish(vel_tp_req);
        geometry_msgs::PoseStamped dummy_pose;
        dummy_pose.header.frame_id = "world_ned";
        dummy_pose.header.stamp = ros::Time::now();
        dummy_pose.pose.orientation.w = 1;
        pubTPdummy.publish(dummy_pose);
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
            // as_.shutdown(); ?
            timer_.stop();
        }
    }

    void update(const ros::TimerEvent &event)
    {

        // std::cout << "controlando\n";
        feedback_.target = setPoint;

        t = tfBuffer.lookupTransform("world_ned", "girona500/origin", ros::Time(0));
        feedback_.current.position.x = t.transform.translation.x;
        feedback_.current.position.y = t.transform.translation.y;
        feedback_.current.position.z = t.transform.translation.z;
        feedback_.current.orientation.x = t.transform.rotation.x;
        feedback_.current.orientation.y = t.transform.rotation.y;
        feedback_.current.orientation.z = t.transform.rotation.z;
        feedback_.current.orientation.w = t.transform.rotation.w;
        tf2::fromMsg(t.transform, mat_curr);

        tf2::fromMsg(setPoint, mat_goal);
        error = mat_curr.inverseTimes(mat_goal);
        tf2::toMsg(error, feedback_.error);

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
        // integral += error.getOrigin().z() * dt.toSec();
        // pid_err(2, 1) = 0.1 * integral;

        // derivative smooths
        derivative = (error.getOrigin().x() - last_error_x) / dt.toSec();
        pid_err(0, 2) = 0.3 * derivative;
        derivative = (error.getOrigin().y() - last_error_y) / dt.toSec();
        pid_err(1, 2) = 0.3 * derivative;
        derivative = (error.getOrigin().z() - last_error_z) / dt.toSec();
        pid_err(2, 2) = 0.3 * derivative;

        last_error_x = error.getOrigin().x();
        last_error_y = error.getOrigin().y();
        last_error_z = error.getOrigin().z();

        // std::cout << "x error:" << setPoint.position.x - t.transform.translation.x << "\n";
        // std::cout << "y error:" << setPoint.position.y - t.transform.translation.y << "\n";
        // std::cout << "z error:" << setPoint.position.z - t.transform.translation.z << "\n";

        // std::cout << "x proportional:" << pid_err(0, 0) << "\n";
        // std::cout << "x integral:" << pid_err(0, 1) << "\n";
        // std::cout << "x derivative:" << pid_err(0, 2) << "\n";
        // std::cout << "x action unclamped:" << pid_err.row(0).sum() << "\n";
        std::cout << "x action:" << std::clamp(pid_err.row(0).sum(), -max_vel, max_vel) << "\n";
        std::cout << "y action:" << std::clamp(pid_err.row(1).sum(), -max_vel, max_vel) << "\n";
        std::cout << "z action:" << std::clamp(pid_err.row(2).sum(), -max_vel, max_vel) << "\n";
        std::cout << "yaw action:" << std::clamp(0.7 * err_yaw, -max_rot_vel, max_rot_vel) << "\n";
        // std::cout << "raw_yaw: " << err_yaw << "\n";

        if (interface == "tp_controller")
        {
            sendVelTP(pid_err, err_yaw);
        }
        else
        {
            sendVelCOLA2(pid_err, err_yaw);
        }

        prev_t = ros::Time::now();
        // ros::Duration(0.1).sleep();
    }
};

int main(int argc, char **argv)
{
    std::cout << "executing path\n";
    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle n;

    PID pid(ros::this_node::getName());

    ros::Subscriber sub = n.subscribe(ros::this_node::getName() + "/vel_target", 1, &PID::changeVel, &pid);

    ros::spin();
}