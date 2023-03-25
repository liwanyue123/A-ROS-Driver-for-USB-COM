/**
 * @Author: Wanyue Li
 * @Date: 2023/3/25 14:13:54
 * @LastEditors: Wanyue Li
 * @LastEditTime: 2023/3/25 14:13:54
 * Description: 
 * Copyright: Copyright (©)}) 2023 Wanyue Li. All rights reserved.
 */
#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#include <ros/ros.h>
#include <serial/serial.h> //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <string>
#include <math.h>
#include "Utilities/Period/PeriodicTask.h"
#include "Utilities/utilities.h"



// 串口通讯类，下面管辖nmea解析类
class ComManager
{
private:
    //----------------ros----------------
    ros::Publisher message_pub; // 读到com消息解析之后，发送ros topic

    //----------------串口设置----------------
    string comName = "/dev/ttyUSB0"; // 默认值，实际值读yaml
    //----------------参数设置----------------
    string messageSavePath = "/home/orin/rawData.txt";//你要改一下
    bool isResetMessageData = false;

    bool isResetConfig = false;
    int baudrate = 460800;
 

    // clock_t startTime, endTime;
    bool isOpen = false;
 
    ros::Publisher rawMessage_pub;

    FILE *fp = NULL;
    ofstream outFile;	//定义ofstream对象outFile
    void init(ros::NodeHandle node)
    {
        updateYamlParam(node); // 读yaml,获取参数
        rawMessage_pub = node.advertise<std_msgs::String>("/rawMessage", 50);//发布原始数据
 
 	 


  // 打开要存报文的文件
        if (isResetMessageData == false)
        {
            // 追加写
            outFile.open(messageSavePath.c_str(),std::ios::app);	
            if (!outFile.is_open())
            {
                printf("Could not find the file: %s\n", messageSavePath.c_str());
                FileOperator::createFile(messageSavePath);
            }
        }
        else
        {
            // 重新写
            outFile.open(messageSavePath.c_str(),std::ios::trunc);	
            if (!outFile.is_open())
            {
                printf("Could not find the file: %s\n", messageSavePath.c_str());
                FileOperator::createFile(messageSavePath);
            }
        }

    }


    void updateYamlParam(ros::NodeHandle node) // 读yaml，获取参数
    {
        node.getParam("/COM_PORT", comName);
        node.getParam("/COM_BAUD", baudrate);
        node.getParam("/COM_SAVE_PATH", messageSavePath);
        node.getParam("/COM_SAVE_MODE_RESET", isResetMessageData);
        node.getParam("/COM_RATE_RESET", isResetConfig);


    }

public:
    ComManager(ros::NodeHandle node)
    {
        init(node);
    }
    ~ComManager()
    {
        // 关闭文件
        StringUtils::closeFile(fp);
        	outFile.close();	

    }
    // 打开串口
    bool openCom();
    bool sendComMessage(string str, int len);
    void analyseStringMessage(string Message, int len_message);

    void listenSingleCom();
    void clearBuffer();
 
    void publishIMUTopic();
    void publishGPSTopic();

    void setComName(string _comName)
    {
        this->comName = _comName;
    }
    string getComName()
    {
        return this->comName;
    }
    void setBaudrate(int _baudrate)
    {
        this->baudrate = _baudrate;
    }
    serial::Serial com; //  声明串口对象
};

// 定时线程类，下面管辖串口通讯类
// 定时调用串口类来监听数据，单独开个线程
class ListenComPeriodicTask : public PeriodicTask
{
public:
    // 传入线程管理者，频率，线程名字
    // 传入can套接字
    ListenComPeriodicTask(PeriodicTaskManager *taskManager, float period, std::string name, int whichCan) : PeriodicTask(taskManager, period, name)
    {
        _whichCom = whichCan;
    }

    void run() override
    {
        //-----获取数据-----
        // cout << "comManager->listenSingleCom()" << endl;
        if (comManager != nullptr)
        {
            comManager->listenSingleCom();
        }
        else
        {
            ROS_ERROR("comManager is not exit");
        }
    }

    void init() override
    {
        _init = true;
    }
    // set串口管理类,一定要先设置这个才去run()
    void setComManager(ComManager *comM)
    {

        this->comManager = comM;
    }

    // 关不掉的，因为socket读取是堵塞式的但是没关系
    void cleanup() override
    {

        // close(s);
        _cleanedUp = true;
    }

private:
    int _whichCom = 0;
    int _counter = 0;
    bool _cleanedUp = false;
    bool _init = false;
    bool _slow = false;
    int nbytes;                       // 发送到数据长度
    ComManager *comManager = nullptr; // 串口通讯类
};

#endif