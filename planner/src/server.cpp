#include <ros/ros.h>
#include <plan_srv/update.h>


bool callback(plan_srv::update::Request& req,
              plan_srv::update::Response &res)
{
    ROS_INFO("update x: %d",req.goal_x);
    ROS_INFO("update y: %d",req.goal_y);
    ROS_INFO("update theta: %d",req.goal_theta);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server", ros::InitOption::AnonymousName);
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("update_goal",callback);
    ROS_INFO("----- service update_goal waiting for request -----");
    ros::spin();

    return 0;
}
