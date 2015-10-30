#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <gtp_ros_msg/requestAction.h>
#include <msgClient.hh>
#include <jsoncpp/json/json.h>

#include <string>
#include <iostream>

#include "std_msgs/String.h"
#include <gtp_ros_msg/GTPTraj.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

msgClient client;
ros::Publisher trajectory;

class GtpRosBridge
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<gtp_ros_msg::requestAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  gtp_ros_msg::requestFeedback feedback_;
  gtp_ros_msg::requestResult result_;
  

  


public:

  GtpRosBridge(std::string name) :
    as_(nh_, "gtp_ros_server", boost::bind(&GtpRosBridge::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~GtpRosBridge(void)
  {
  }

  void executeCB(const gtp_ros_msg::requestGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    ROS_INFO("%s: Computing the request", action_name_.c_str());
    

    //int f = str.find("taskAlternativeId");
    //std::cout << str.c_str()[f] << std::endl; //163
    //int vir = str.rfind(",");
    //std::cout << vir << std::endl; //164
    


        
        
    //std::string altIdStr = str.substr(str.find("taskAlternativeId") + 19, str.rfind(",") - str.find("taskAlternativeId") - 19);
    //std::string IdStr = str.substr(str.find("taskId") + 8, str.rfind("}") - 1 - str.find("taskId") - 8);
    
    Json::FastWriter wrt;
    if (goal->req.requestType == "planning")
    {
        client.sendMessage("move3d", "{\"ClearGTPInputs\":null}");
        std::cout << client.getBlockingMessage().second << std::endl;
        
        
        //_result = wrt.write(res);

        
        for (unsigned int i = 0; i< goal->req.involvedAgents.size(); i++)
        {
           Json::Value request(Json::objectValue);
           Json::Value input(Json::objectValue);
           
           input["key"] = goal->req.involvedAgents.at(i).actionKey;
           input["value"] = goal->req.involvedAgents.at(i).agentName;
           request["AddGTPAgent"] = input;
           
           client.sendMessage("move3d", wrt.write(request));
           ROS_INFO("planner answer: %s", client.getBlockingMessage().second.c_str());
        }
        
        for (unsigned int i = 0; i< goal->req.involvedObjects.size(); i++)
        {
           Json::Value request(Json::objectValue);
           Json::Value input(Json::objectValue);
           
           input["key"] = goal->req.involvedObjects.at(i).actionKey;
           input["value"] = goal->req.involvedObjects.at(i).objectName;
           request["AddGTPObject"] = input;
           
           client.sendMessage("move3d", wrt.write(request));
           ROS_INFO("planner answer: %s", client.getBlockingMessage().second.c_str());
        }
        
        for (unsigned int i = 0; i< goal->req.data.size(); i++)
        {
           Json::Value request(Json::objectValue);
           Json::Value input(Json::objectValue);
           
           input["key"] = goal->req.data.at(i).dataKey;
           input["value"] = goal->req.data.at(i).dataValue;
           request["AddGTPData"] = input;
           
           client.sendMessage("move3d", wrt.write(request));
           ROS_INFO("planner answer: %s", client.getBlockingMessage().second.c_str());
        }
        
        for (unsigned int i = 0; i< goal->req.points.size(); i++)
        {
        
           Json::Value request(Json::objectValue);
           Json::Value input(Json::objectValue);
           
           input["x"] = goal->req.points.at(i).value.x;
           input["y"] = goal->req.points.at(i).value.y;
           input["z"] = goal->req.points.at(i).value.z;
           
           input["key"] = goal->req.points.at(i).pointKey;
           request["AddGTPPoint"] = input;
           
           client.sendMessage("move3d", wrt.write(request));
           ROS_INFO("planner answer: %s", client.getBlockingMessage().second.c_str());
        }
        
        Json::Value request(Json::objectValue);
        Json::Value input(Json::objectValue);
        
        input["computeMotionPlan"] = true;
        input["previousTaskAlternativeId"] = (int)goal->req.predecessorId.alternativeId;
        input["previousTaskId"] = (int)goal->req.predecessorId.actionId;
        input["type"] = goal->req.actionName;
        request["PlanGTPTask"] = input;
        
        client.sendMessage("move3d", wrt.write(request));
        
        std::string str = client.getBlockingMessage().second;
        //ROS_INFO("%s",str.c_str());
        
        Json::Value root,nullval,tmpVal;
        Json::Reader reader;
        
        bool parsedSuccess = reader.parse(str, root, false);
        if(!parsedSuccess || root.size() <= 0)
        {
            ROS_INFO("Failed to parse JSON");
            result_.ans.success = false;
            as_.setSucceeded(result_);
            return;
        }
        
        
        
        tmpVal = root.get("PlanGTPTask", nullval );
        if (tmpVal == nullval)
        {
           ROS_INFO("Failed to find PlanGTPTask");
           result_.ans.success = false;
           as_.setSucceeded(result_);
           return;
        }
        

        std::string report = tmpVal.get("returnReport", "Null" ).asString();
        
        
        if (report == "OK")
           result_.ans.success = true;
        else
        {
           ROS_INFO("Failed to compute, reason: %s", report.c_str());
           result_.ans.success = false;
           as_.setSucceeded(result_);
           return;
        }
          
        result_.ans.identifier.actionId = tmpVal.get("taskId", "Null" ).asInt();
        result_.ans.identifier.alternativeId = tmpVal.get("taskAlternativeId", "Null" ).asInt();
        
        
        ROS_INFO("planner answer: %s", str.c_str());
        as_.setSucceeded(result_);
    }
    else if (goal->req.requestType == "details")
    {
        Json::Value request(Json::objectValue);
        Json::Value input(Json::objectValue);
        
        input["taskId"] = (int)goal->req.loadAction.actionId;
        input["alternativeId"] = (int)goal->req.loadAction.alternativeId;
        request["GetGTPDetails"] = input;
        
        client.sendMessage("move3d", wrt.write(request));
        std::string res = client.getBlockingMessage().second;
        //ROS_INFO("planner answer: %s", res.c_str());
        
        
        Json::Value root, answer, nullval;
        Json::Reader reader;
  
        reader.parse(res, root, false);
        answer = root.get("GetGTPDetails", nullval );
        
        if (answer == nullval)
        {
          ROS_INFO("no GetGTPDetails");
          result_.ans.success = false;
          as_.setSucceeded(result_);
          return;
        }
        
        if (answer.get("status", "NULL").asString() != "OK")
        {
          ROS_INFO("get details failed");
          result_.ans.success = false;
          as_.setSucceeded(result_);
          return;
        }
        
        const Json::Value& subTrajs = answer["subTrajs"];
        result_.ans.subTrajs.clear();
        for (Json::ValueConstIterator it = subTrajs.begin(); it != subTrajs.end(); ++it)
        {
            const Json::Value& subTraj = *it;
            // rest as before
            gtp_ros_msg::SubTraj subMsg;
            subMsg.agent = subTraj["agent"].asString();
            subMsg.armId = subTraj["armId"].asInt();
            subMsg.subTrajId = subTraj["subTrajId"].asInt();
            subMsg.subTrajName = subTraj["subTrajName"].asString();
            result_.ans.subTrajs.push_back(subMsg);
        }
        result_.ans.success = true;
        as_.setSucceeded(result_);
        
    }
    else if (goal->req.requestType == "load")
    {
        Json::Value request(Json::objectValue);
        Json::Value input(Json::objectValue);
        
        input["taskId"] = (int)goal->req.loadAction.actionId;
        input["alternativeId"] = (int)goal->req.loadAction.alternativeId;
        input["subTrajId"] = (int)goal->req.loadAction.alternativeId;
        request["GetGTPTraj"] = input;
        
        client.sendMessage("move3d", wrt.write(request));
        std::string res = client.getBlockingMessage().second;
        //ROS_INFO("planner answer: %s", res.c_str());
        
        
        Json::Value root, nullval;
        Json::Reader reader;
        
        reader.parse(res, root, false);
        
        gtp_ros_msg::GTPTraj t;
        for (unsigned int i = 0; i < root["GetGTPTraj"]["confs"].size(); i++)
        {
            trajectory_msgs::JointTrajectoryPoint Jtpt;
            for (unsigned int j = 0; j < root["GetGTPTraj"]["confs"][i].size(); j++)
            {
               Jtpt.positions.push_back(root["GetGTPTraj"]["confs"][i][j].asDouble());
            }
            t.traj.points.push_back(Jtpt);
        }
        
        
        t.name = "traj";
        
       
  
        t.traj.joint_names.push_back("dummy0");
        t.traj.joint_names.push_back("dummy1");
        t.traj.joint_names.push_back("dummy2");
        t.traj.joint_names.push_back("dummy3");
        t.traj.joint_names.push_back("dummy4");
        t.traj.joint_names.push_back("dummy5");
        t.traj.joint_names.push_back("navX");
        t.traj.joint_names.push_back("navY");
        t.traj.joint_names.push_back("dummyZ");
        t.traj.joint_names.push_back("dummyRX");
        t.traj.joint_names.push_back("dummyRY");
        t.traj.joint_names.push_back("RotTheta");
        t.traj.joint_names.push_back("torso_lift_joint");
        t.traj.joint_names.push_back("head_pan_joint");
        t.traj.joint_names.push_back("head_tilt_joint");
        t.traj.joint_names.push_back("laser_tilt_mount_joint");
        
        t.traj.joint_names.push_back("r_shoulder_pan_joint");
        t.traj.joint_names.push_back("r_shoulder_lift_joint");
        t.traj.joint_names.push_back("r_upper_arm_roll_joint");
        t.traj.joint_names.push_back("r_elbow_flex_joint");
        t.traj.joint_names.push_back("r_forearm_roll_joint");
        t.traj.joint_names.push_back("r_wrist_flex_joint");
        t.traj.joint_names.push_back("r_wrist_roll_joint");
        t.traj.joint_names.push_back("r_gripper_joint");
        
        t.traj.joint_names.push_back("dummyGripper");
        
        
        t.traj.joint_names.push_back("l_shoulder_pan_joint");
        t.traj.joint_names.push_back("l_shoulder_lift_joint");
        t.traj.joint_names.push_back("l_upper_arm_roll_joint");
        t.traj.joint_names.push_back("l_elbow_flex_joint");
        t.traj.joint_names.push_back("l_forearm_roll_joint");
        t.traj.joint_names.push_back("l_wrist_flex_joint");
        t.traj.joint_names.push_back("l_wrist_roll_joint");
        t.traj.joint_names.push_back("l_gripper_joint");
        
        
        
        
        //t.traj.points.
        trajectory.publish(t);
        
        result_.ans.success = true;
        as_.setSucceeded(result_);
        
    }
    else if (goal->req.requestType == "update")
    {
       
    }
    else
    {
        ROS_INFO("request type: \"%s\" not supported, the supported types are: planning, details, load and update", goal->req.requestType.c_str());
        result_.ans.success = false;
        result_.ans.identifier.actionId = -1;
        result_.ans.identifier.alternativeId = -1;
        as_.setSucceeded(result_);
    }
        
  }


};



//void chatterCallback(const std_msgs::String::ConstPtr& msg)
//{
//  ROS_INFO("I heard: [%s]", msg->data.c_str());
//}



int main(int argc, char** argv)
{
  std::string server = "localhost";
  int port = 5500;
  ROS_INFO("Starting node GTP_ROS with server: %s and port: %d",server.c_str(),port);
  

  ros::init(argc, argv, "gtp_ros_server_node");
  
  ros::NodeHandle n;
  trajectory = n.advertise<gtp_ros_msg::GTPTraj>("gtp_trajectory", 1000);
  
  ros::Rate loop_rate(10);
  


//  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  client.connect("gtp_ros_node", server, port);
  if(client.isConnected())
  {
     client.sendMessage("move3d", "{\"InitGTP\":null}");
     bool testMove3d = false;
     int nbTestTime = 0;
     int nbTestType = 0;
     int maxtest = 3;
     while (!testMove3d && nbTestTime < maxtest && nbTestType < maxtest)
     {
        std::pair<std::string,std::string> ans = client.getMessageTimeout(2000);
        if (ans.second==STATUS_TIMEOUT)
        {
           nbTestTime++;
           ROS_INFO("The server timed out %d times", nbTestTime);
           continue;
        }
        if (ans.first != "move3d")
        {
           nbTestType++;
           continue;
        }
        testMove3d = true;

        GtpRosBridge bridge(ros::this_node::getName());
        ros::spin();
     }
     if (!testMove3d)
     {
        if (nbTestTime >= maxtest)
            ROS_INFO("no answer from move3d (time out reached): is it launched?");
        if (nbTestType >= maxtest)
            ROS_INFO("answer from other modules, might reconsider this code!!");
     }

  }
  else
  {
     ROS_INFO("Failed to connect to geometric planner, abort");
  }
  
  
  


  return 0;
}
