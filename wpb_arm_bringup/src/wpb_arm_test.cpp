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
#include "driver/WPB_Arm_Driver.h"
#include <math.h>
#include <vector>

using namespace std;

typedef struct
{
    float position[7];
    int velocity[7];
}st_wpm_pose;

static CWPB_Arm_Driver wpb_arm;
static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static double pos_send[7];
static int vel_send[7];
static vector<st_wpm_pose> arPose;
static vector<string> arInfo;

void InitPoints()
{
    st_wpm_pose tmpPose;
    //?????????
    arInfo.push_back("????????????");
    memset(&tmpPose,0,sizeof(st_wpm_pose));
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);

    //?????????1?????????
    arInfo.push_back("??????1???90???");
    tmpPose.position[0] = 90;
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);
    //?????????
    arInfo.push_back("??????1?????????");
    memset(&tmpPose,0,sizeof(st_wpm_pose));
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);

    //?????????2?????????
    arInfo.push_back("??????2???10???");
    tmpPose.position[1] = 10;
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);
    //?????????
    arInfo.push_back("??????2?????????");
    memset(&tmpPose,0,sizeof(st_wpm_pose));
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);

     //?????????3?????????
    arInfo.push_back("??????3???30???");
    tmpPose.position[2] = 30;
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);
    //?????????
    arInfo.push_back("??????3?????????");
    memset(&tmpPose,0,sizeof(st_wpm_pose));
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);

     //?????????4?????????
    arInfo.push_back("??????4???30???");
    tmpPose.position[3] = 30;
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);
    //?????????
    arInfo.push_back("??????4?????????");
    memset(&tmpPose,0,sizeof(st_wpm_pose));
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);

     //?????????5?????????
    arInfo.push_back("??????5???30???");
    tmpPose.position[4] = 30;
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);
    //?????????
    arInfo.push_back("??????5?????????");
    memset(&tmpPose,0,sizeof(st_wpm_pose));
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);

     //?????????6?????????
    arInfo.push_back("??????6???30???");
    tmpPose.position[5] = 30;
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);
    //?????????
    arInfo.push_back("??????6?????????");
    memset(&tmpPose,0,sizeof(st_wpm_pose));
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);

    //????????????
    arInfo.push_back("????????????");
    tmpPose.position[6] = 10000;
    arPose.push_back(tmpPose);
    //?????????
    arInfo.push_back("????????????");
    memset(&tmpPose,0,sizeof(st_wpm_pose));
    tmpPose.position[6] = 35000;
    arPose.push_back(tmpPose);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_arm_test");
    ros::NodeHandle n;

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ftdi");
    wpb_arm.Open(strSerialPort.c_str(),115200);

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

    ros::Rate r(30);

    r.sleep();
    for(int i=0;i<7;i++)
    {
        pos_send[i] = 0;
        vel_send[i] = 2000;
    }
    pos_send[6] = 25000;//??????
    wpb_arm.SetJoints(pos_send,vel_send);
    int nCount = 0;
    int nPointIndex = 0;
    InitPoints();

    while(n.ok())
    {
        wpb_arm.ReadNewData();
        wpb_arm.nParseCount ++;
        //ROS_INFO("[wpb_arm.nParseCount]= %d",wpb_arm.nParseCount);
        
        msg.header.stamp = ros::Time::now();
        msg.header.seq ++;

        double fTmp = 0;
        for(int i=0;i<6;i++)
        {
            fTmp = wpb_arm.nRecvJointPos[i];
            fTmp *= 0.01;
            joint_pos[i] = fTmp*fDegToAng;
        }
        fTmp = 38000 - wpb_arm.nRecvJointPos[6];
        if(fTmp < 0) fTmp = 0;
        fTmp *= 0.5/38000;
        joint_pos[6] = fTmp;

        msg.name = joint_name;
        msg.position = joint_pos;
        joint_state_pub.publish(msg);

        //????????????????????????
        nCount ++;
        if(nCount > 150)
        {
            nCount = 0;
            nPointIndex ++;
            if(nPointIndex >= arPose.size())
            {
                nPointIndex = 0;
            }
            for(int i=0;i<7;i++)
            {
                pos_send[i] = arPose[nPointIndex].position[i];
            }
            wpb_arm.SetJoints(pos_send,vel_send);
            printf("[wpb_arm_test]  %d - %s \n",nPointIndex,arInfo[nPointIndex].c_str());
        }

        ros::spinOnce();
        r.sleep();
    }
}