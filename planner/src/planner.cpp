#include <ros/ros.h>
#include <pos_msg/pos.h>
#include <plan_srv/plan.h>
#include <signal.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "AStar.h"
using namespace std;

static map<string, nav_msgs::Path> agent_path;
AStar::Generator generator;

void mySigintHandler(int sig)
{
    ROS_INFO("Exit from planner node !");
    ros::shutdown();
    ros::waitForShutdown();
}

boost::shared_ptr<pos_msg::pos const> subscribeOnce(std::string topic)
{
    boost::shared_ptr<pos_msg::pos const> edge;
    edge = ros::topic::waitForMessage<pos_msg::pos>(topic, ros::Duration(20));
    if(edge != NULL)
    {
    }
    else
        cout<<"no topic agent_feedback"<<endl;
    return edge;
}

nav_msgs::Path calculate_path(boost::shared_ptr<pos_msg::pos const> start, plan_srv::plan::Request& goal)
{
    std::cout << "Generate path ... \n";
    nav_msgs::Path plan;
    AStar::Vec2i startVec,goalVec;
    startVec.x = start->x;
    startVec.y = start->y;
    goalVec.x = goal.goal_x;
    goalVec.y = goal.goal_y;
    
    auto path = generator.findPath(startVec, goalVec);

//       for(auto& coordinate : path)
//       {
//           std::cout << coordinate.x << " " << coordinate.y << "\n";
//       }
    unsigned int size = path.size();
    ROS_INFO("path size: %d", size);
    if(size==0)
    {
        ROS_WARN("sorry, A-star didn't find path");
        return plan;
    }
    plan.poses.reserve(size);

    geometry_msgs::PoseStamped plan_pose;
    for(int i=size-1;i>=0;i--)
    {
        plan_pose.header.seq = size-i;
        plan_pose.header.stamp =ros::Time::now();
        plan_pose.header.frame_id = "agent_plan";

        plan_pose.pose.position.x = path.at(i).x;
        plan_pose.pose.position.y = path.at(i).y;
        plan_pose.pose.position.z = 0;
        plan_pose.pose.orientation.x = 0;
        plan_pose.pose.orientation.y = 0;
        plan_pose.pose.orientation.z = 0;
        plan_pose.pose.orientation.w = 0;

        plan.header = plan_pose.header;
        plan.poses.push_back(plan_pose);
    }
    return plan;
}

bool callback(plan_srv::plan::Request& req,
              plan_srv::plan::Response &res)
{
    boost::shared_ptr<pos_msg::pos const> startPos;

    startPos = subscribeOnce(req.id+"/agent_feedback");
    if(!startPos)
    {
        ROS_ERROR("planner receives %s start position failed !",req.id.c_str());
        return false;
    }
    ROS_INFO("%s, start x: %d",req.id.c_str(), startPos->x);
    ROS_INFO("%s, start y: %d",req.id.c_str(), startPos->y);
    ROS_INFO("%s, start theta: %d",req.id.c_str(), startPos->theta);

    res.plan = calculate_path(startPos, req);
    agent_path[req.id] = res.plan;
    return true;
}

int main(int argv, char** argc)
{
    ros::init(argv,argc,"planner_node");
    ros::NodeHandle nh;
    signal(SIGINT,mySigintHandler);

    generator.setWorldSize({10, 10});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    ros::ServiceServer server = nh.advertiseService("get_plan",callback);
    ROS_INFO("----- waiting for agent's request -----");
    ros::spin();
    return 0;
}
