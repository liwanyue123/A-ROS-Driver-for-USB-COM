/**
 * @Author: Wanyue Li
 * @Date: 2023/3/25 14:13:33
 * @LastEditors: Wanyue Li
 * @LastEditTime: 2023/3/25 14:13:33
 * Description: 
 * Copyright: Copyright (©)}) 2023 Wanyue Li. All rights reserved.
 */
#include "ComManager.h"
 

using namespace std;
const int comTotalNum = 1;

//----------------主函数----------------
int main(int argc, char **argv)
{

    //-----------------ROS初始化--------------------
    // 初始化节点
    ros::init(argc, argv, "COM_node");
    // 声明节点句柄

    ros::NodeHandle n;
    //-----------------串口初始化--------------------
    string comNameList[comTotalNum] = {"/dev/ttyUSB0"}; //
    ComManager *comManager[comTotalNum];                // 串口通讯类 comTotalNum
    for (int i = 0; i < comTotalNum; i++)
    {
        comManager[i] = new ComManager(n);//初始化
        comManager[i]->setComName(comNameList[i]); // 设置串口名字
        comManager[i]->openCom();                  // 115200 921600
        ros::Duration(0.1).sleep();
        if (comManager[i]->com.isOpen()) // 检测串口是否已经打开,并给出提示信息
        {
            ROS_INFO("Serial Port%d initialized\n", i);
    
        }
        else
        {
            return -1;
        }
    }

    //---------------开启监听usb数据线程---------------
    PeriodicTaskManager taskManager;
    ListenComPeriodicTask *taskPool[comTotalNum]; // n个串口监听线程

    for (int i = 0; i < comTotalNum; i++)
    {
        string periodName = "Listen-" + comManager[i]->getComName();
        taskPool[i] = new ListenComPeriodicTask(&taskManager, 0.0001f, periodName, i); // 10000hz
        taskPool[i]->setComManager(comManager[i]);                                     // 将串口分析类传入监听线程中，这样串口有数据时就给他解析

        periodName += to_string(i);

        taskPool[i]->start();
    }

    ROS_INFO("Start to listen Com\n");
    ros::spin(); // 收到主程序的指令后，就调用controlMotor_callback组织发送报文
    taskManager.closeAll();
    return 0;
}
