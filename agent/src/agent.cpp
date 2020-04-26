#include <ros/ros.h>
#include <pos_msg/pos.h>
#include <plan_srv/plan.h>
#include <string>
#include <boost/thread.hpp>
#include <nav_msgs/Path.h>
#include <iostream>
#include <signal.h>
using namespace std;


void mySigintHandler(int sig)
{
    ROS_INFO("Exit from agent node !");
    ros::shutdown();
    ros::waitForShutdown();
}

void serviceRequest(ros::NodeHandle nh, plan_srv::plan srv)
{
    ros::ServiceClient client = nh.serviceClient<plan_srv::plan>("/get_plan");
    while(!client.exists())
    {
        ROS_WARN("client %s is invalid, waiting for service get_plan", srv.request.id.c_str());
        sleep(1);
    }
    if(client.call(srv))
    {
        ROS_INFO("requesting plan for goal !");
    }
    else {
        ROS_ERROR("requesting plan went wrong !");
    }

    ROS_INFO("receive motion plan from planner:");
    unsigned int size = srv.response.plan.poses.size();
    if(size>0)
    {
        for(int i=0; i<size; i++)
            cout << srv.response.plan.poses.at(i) <<endl;
        ros::Publisher gui_pub = nh.advertise<nav_msgs::Path>("gui_path",100,false);
 	ros::Rate rate(1000);
	while(ros::ok())
        {
            gui_pub.publish(srv.response.plan);
	}
    }
    else
    {
        ROS_WARN("planner couldn't generate a path");
    }
}

int main(int argv, char** argc)
{
    ros::init(argv,argc,"agent_node");
    ros::NodeHandle nh("~");
    signal(SIGINT, mySigintHandler);
    std::string agent_id;
    int x,y,theta;
    int goal_x, goal_y, goal_theta;
    nh.param("id",agent_id,std::string("agent") );

    nh.param("x",x,0);
    nh.param("y",y,0);
    nh.param("theta",theta,0);

    nh.param("goal_x",goal_x,0);
    nh.param("goal_y",goal_y,0);
    nh.param("goal_theta",goal_theta,0);

    pos_msg::pos msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;

    plan_srv::plan srv;
    srv.request.id = agent_id;
    srv.request.goal_x = goal_x;
    srv.request.goal_y = goal_y;
    srv.request.goal_theta = goal_theta;

    boost::thread client = boost::thread(boost::bind(&serviceRequest, nh, srv) );

    ros::Publisher pub = nh.advertise<pos_msg::pos>("agent_feedback",10,false);
    ros::Rate rate(10);
    while(ros::ok())
    {
        pub.publish(msg);
        rate.sleep();
    }
    return 0;
}
