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
#include <sensor_msgs/JointState.h>
#include <opencv2/highgui/highgui.hpp> 

using namespace cv;

static int joint_angle[6];

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_arm_trackbars");
   
    ros::NodeHandle n;
    ros::Publisher joint_ctrl_pub = n.advertise<sensor_msgs::JointState>("wpb_arm/joint_ctrl_degree", 30);

    //控制变量
    sensor_msgs::JointState ctrl_msg;
    ctrl_msg.name.resize(6);
    ctrl_msg.position.resize(6);
    ctrl_msg.velocity.resize(6);
    //关节角度
    for(int i=0;i<6;i++)
    {
        joint_angle[i] = 90;
        ctrl_msg.position[i] = joint_angle[i];
    }

    ros::Rate loop_rate(10);

    //生成图像显示和参数调节的窗口空见
    namedWindow("Pannel", WINDOW_AUTOSIZE);

    createTrackbar("Joint_0", "Pannel", &(joint_angle[0]), 180); //角度范围 (0 - 180)
    createTrackbar("Joint_1", "Pannel", &(joint_angle[1]), 180); //角度范围 (0 - 180)
    createTrackbar("Joint_2", "Pannel", &(joint_angle[2]), 180); //角度范围 (0 - 180)
    createTrackbar("Joint_3", "Pannel", &(joint_angle[3]), 180); //角度范围 (0 - 180)
    createTrackbar("Joint_4", "Pannel", &(joint_angle[4]), 180); //角度范围 (0 - 180)
    createTrackbar("Joint_5", "Pannel", &(joint_angle[5]), 90); //角度范围 (0 - 90)
    while( ros::ok())
    {
        cv::waitKey(1);
        for(int i=0;i<6;i++)
        {
            ctrl_msg.position[i] = joint_angle[i];
        }
        joint_ctrl_pub.publish(ctrl_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}