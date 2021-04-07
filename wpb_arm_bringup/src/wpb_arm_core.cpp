/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include "driver/WPB_Arm_Driver.h"
#include <math.h>
#include <vector>

using namespace std;

typedef struct
{
    float position[7];
    int velocity[7];
}st_wpm_pose;

typedef struct
{
    float fGapSize;
    int nGripperPos;
    double fPosePerMM;
}stGripperPos;
static vector<stGripperPos> arGripperPos;

static CWPB_Arm_Driver wpb_arm;
static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static double pos_send[6];
static int vel_send[6];
static st_wpm_pose tmpPose;
static vector<st_wpm_pose> arPose;
static int nExecIndex = 0;
static bool bExecPath = false;
static bool bExecToGoal = true;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;

// 响应 Move_Group 的轨迹执行
void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, TrajectoryServer* as)
{
    int nrOfPoints = goal->trajectory.points.size();
    // ROS_INFO("Trajectory with %d positions received \n", nrOfPoints);

    nExecIndex = nrOfPoints - 1;
    //获取关节角度
    for(int i=0;i<5;i++)
    {
        pos_send[i] = goal->trajectory.points[nExecIndex].positions[i] * fAngToDeg + 90;
    }
    pos_send[4] = 180 -pos_send[4];
    // pos_send[2] = 90;
    
     ROS_WARN(" name 1-%s 2-%s 3-%s 4-%s 5-%s ",
        goal->trajectory.joint_names[0].c_str(),
        goal->trajectory.joint_names[1].c_str(),
        goal->trajectory.joint_names[2].c_str(),
        goal->trajectory.joint_names[3].c_str(),
        goal->trajectory.joint_names[4].c_str());
    
    int num_joints = goal->trajectory.joint_names.size();
      ROS_WARN(" pos %.1f %.1f %.1f %.1f %.1f ",
        pos_send[0],
        pos_send[1],
        pos_send[2],
        pos_send[3],
        pos_send[4]);
    
    wpb_arm.SetJoints(pos_send,vel_send);

    as->setSucceeded();
}

void InitGripperPosVal()
{
    stGripperPos tmpGP;
    tmpGP.fGapSize = 0;
    tmpGP.nGripperPos = 38000;
    tmpGP.fPosePerMM = 0;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.006;
    tmpGP.nGripperPos = 35000;
    arGripperPos.push_back(tmpGP);

    int nNumGP = arGripperPos.size();
    for(int i=1;i<nNumGP;i++)
    {
        double fDiffSize = fabs(arGripperPos[i].fGapSize - arGripperPos[i-1].fGapSize);
        int nDiffPos = fabs(arGripperPos[i].nGripperPos - arGripperPos[i-1].nGripperPos);
        arGripperPos[i].fPosePerMM = fDiffSize/nDiffPos;
    }
}

// 手爪位置计算
int CalGripperPos(float inGapSize)
{
    int nNumGP = arGripperPos.size();
    int nRetGripperPos = 0;
    if(nNumGP > 0)
    {
        int nIndexGP = nNumGP - 1;
        if(inGapSize >= arGripperPos[nIndexGP].fGapSize)
        {
            nRetGripperPos = arGripperPos[nIndexGP].nGripperPos;
            return nRetGripperPos;
        }
        for(int i=1;i<nNumGP;i++)
        {
            if(inGapSize < arGripperPos[i].fGapSize)
            {
                nIndexGP = i;
                break;
            }
        }
        if(nIndexGP < nNumGP)
        {
            double fDiffGapSize = fabs(inGapSize - arGripperPos[nIndexGP].fGapSize);
            int nDiffGripperPos = (fDiffGapSize/arGripperPos[nIndexGP].fPosePerMM);
            nRetGripperPos = arGripperPos[nIndexGP].nGripperPos + nDiffGripperPos;
        }
    }
    return nRetGripperPos;
}

//角度控制机械臂
void JointCtrlDegreeCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    bExecPath = false;
    int nNumJoint = msg->position.size();
    if(nNumJoint > 7)
    {
        nNumJoint = 7;
    }
    for(int i=0;i<nNumJoint;i++)
    {
        pos_send[i] = msg->position[i];
        vel_send[i] = msg->velocity[i];
        ROS_INFO("[JointCtrlDegreeCallback] %d - %s = %.2f", i, msg->name[i].c_str(),msg->position[i]);
    }
    wpb_arm.SetJoints(pos_send,vel_send);
}

//弧度控制机械臂
void JointCtrlRadianCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    bExecPath = false;
    int nNumJoint = msg->position.size();
    if(nNumJoint > 7)
    {
        nNumJoint = 7;
    }
    for(int i=0;i<nNumJoint;i++)
    {
        if(i != 6)
        pos_send[i] = msg->position[i] * fAngToDeg;
        else
        pos_send[i] = msg->position[i]; //手爪
        vel_send[i] = msg->velocity[i];
        ROS_INFO("[JointCtrlRadianCallback] %d - %s = %.2f (%.0f Deg)", i, msg->name[i].c_str(),msg->position[i],pos_send[i]);
    }
    wpb_arm.SetJoints(pos_send,vel_send);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_arm_core");
    ros::NodeHandle n;

    ros::Subscriber joint_ctrl_degree_sub = n.subscribe("/wpb_arm/joint_ctrl_degree",30,&JointCtrlDegreeCallback);
    ros::Subscriber joint_ctrl_radian_sub = n.subscribe("/wpb_arm/joint_ctrl_radian",30,&JointCtrlRadianCallback);
    
	TrajectoryServer tserver(n, "wpb_arm_controller/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &tserver), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
    tserver.start();

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ch34x");
    wpb_arm.Open(strSerialPort.c_str(),115200);
    n_param.param<bool>("exec_to_goal", bExecToGoal, true);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_name(5);
    std::vector<double> joint_pos(5);
 
    joint_name[0] = "joint_0";
    joint_name[1] = "joint_1";
    joint_name[2] = "joint_2";
    joint_name[3] = "joint_3";
    joint_name[4] = "joint_4";
    joint_pos[0] = 0.0f;
    joint_pos[1] = 0.0f;
    joint_pos[2] = 0.0f;
    joint_pos[3] = 0.0f;
    joint_pos[4] = 0.0f;
    pos_send[5] = 90;
    
    ros::Rate r(30);

    r.sleep();
    int nCount = 0;

    while(n.ok())
    {
        wpb_arm.ReadNewData();
        //ROS_INFO("[wpb_arm.nParseCount]= %d",wpb_arm.nParseCount);
        
        msg.header.stamp = ros::Time::now();
        msg.header.seq ++;

        double fTmp = 0;
        for(int i=0;i<6;i++)
        {
            fTmp = wpb_arm.nRecvJointPos[i];
            joint_pos[i] = (fTmp-90)*fDegToAng;
        }
        // joint_pos[2] =  -joint_pos[2];
        joint_pos[4] =  -joint_pos[4];
        msg.name = joint_name;
        msg.position = joint_pos;
        joint_state_pub.publish(msg);

        ros::spinOnce();
        r.sleep();
    }
}