/**
 * @Author: Wanyue Li
 * @Date: 2023/3/25 14:13:30
 * @LastEditors: Wanyue Li
 * @LastEditTime: 2023/3/25 14:13:30
 * Description: 
 * Copyright: Copyright (©)}) 2023 Wanyue Li. All rights reserved.
 */
#include "ComManager.h"

// 把一个字符串按照delim切割成部分，存入str中
int splitStringTo2Part(const std::string &line, const std::string &delim, std::vector<std::string> &vstr)
{

    int phit = 0;
    std::string sstr;
    int length = line.length();

    vstr.clear();

    phit = line.rfind(delim); // delim是",",phit是，在原报文中的位置
    // phit+=2;
    if (phit != -1)
    {
        /* find delim, get substr */
        sstr = line.substr(0, phit + delim.size()); // 切开[0,5)表示[0,4]
        vstr.push_back(sstr);                       // 存入堆栈

        sstr = line.substr(phit + delim.size(), length); // 切开
        vstr.push_back(sstr);
    }
    else
    {
        /* not find delim, append remaining str and break */
        vstr.push_back(line.substr(0));
        return -1;
    }

    return vstr.size();
}

// 打开串口
bool ComManager::openCom()
{

    serial::Timeout to = serial::Timeout::simpleTimeout(100); // 超时设置
    try
    {
        // 设置串口属性,并打开串口
        // 默认波特率：9600  parity:none dataBit:8 stopBit:1

        string command_down = "sudo chmod 777 " + comName;
        system(const_cast<char *>(command_down.c_str()));

        com.setPort(comName);
        com.setBaudrate(baudrate);
        com.setTimeout(to);
        com.open();
        ROS_INFO("%s is open\n", comName.c_str());
        isOpen = true;
        return true;
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        isOpen = false;
        return false;
    }
}

// 向对应串口发送报文
bool ComManager::sendComMessage(string str, int len)

{
    cout << "---------------send message---------------" << endl;
    cout << "str: " << str << endl;
    const uint8_t *m_str = reinterpret_cast<const uint8_t *>(str.c_str());
    com.write(m_str, len); // 发送数据
}

// 解析一整条报文
void ComManager::analyseStringMessage(string Message, int singleMesLength)
{
    // 打印看看
    cout << Message << endl;
    // 写进文件里面
    // StringUtils::continueWriteFile(fp, completeMessage);
    outFile << Message << endl;
    // ——————————————解析报文 Parsing——————————————
    // You should put your parsing code of the message here

    // ——————————————发送ROS topic——————————————
    // 发布原始数据
    std_msgs::String rawMsg;
    rawMsg.data = Message;
    rawMessage_pub.publish(rawMsg);
}

/*
主体程序运行
*/
// 时间统计
// typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
// typedef std::chrono::duration<float> duration;
string remainMessage = "";   // 一条报文\r\n之后的部分
string completeMessage = ""; // 完整的一条报文
// 只读取某一个串口信息
void ComManager::listenSingleCom()
{

    int totalMesLength = 0; // 完整一条报文的长度
    bool isFindEnd = false; // 是否找到\r\n了

    if (isOpen == false)
    {

        ROS_ERROR_STREAM("COM is not open\n");
    }

    // #define COM_NMEA_LINE_SEP "\r\n"
    //  std::string mesLinesep = COM_NMEA_LINE_SEP;       //"\n"

    // 无论多少报文一次读完
    // std_msgs::String result;
    string result;
    // isFindEnd = false;
    int singleMesLength = 0;
    int count = 0;

    while (!isFindEnd)
    {

        //-----------------读取电机返回内容--------------------
        singleMesLength = com.available(); // 收到的数据长度

        if (singleMesLength) // 有数据来了
        {
            // cout << endl;
            // cout << "---------------------------------" << endl;
            // printf("com's data length=%d\n", singleMesLength);

            std::vector<std::string> mesLines;

            result = com.read(singleMesLength); // 读空缓存 这个效率高很多很多

            (void)splitStringTo2Part(result, "\r\n", mesLines); // 分割字符串
                                                                // (void)splitString(result, "*", mesLines); // 分割字符串

            // \r\n之后的部分存为remain，合并进下一次
            // \r\n之前的部分合并进这一次
            if (mesLines.size() >= 2)
            {
                remainMessage = mesLines.back(); // \r\n之后的部分存为remain，合并进下一次

                for (int i = 0; i < mesLines.size() - 1; i++)
                {
                    completeMessage = completeMessage + mesLines[i]; // \r\n之前的部分合并进这一次
                }
                isFindEnd = true;
                // cout << "rrnn" << endl;
            }
            else // 有数据，但是没有隔断,合并进这一次
            {
                completeMessage = completeMessage + result;
                count++;
                isFindEnd = false;
                ros::Duration(0.002).sleep();
            }
            totalMesLength = totalMesLength + singleMesLength;
        }
        else // 没有消息，也停止
        {
            // isFindEnd = true;
        }
        // cout << "again" << endl;
    }

    if (totalMesLength != 0 && isFindEnd == true)
    {

        analyseStringMessage(completeMessage, totalMesLength); // 分析数据

        // 清空，为下一次读取做准备
        completeMessage = remainMessage;
        remainMessage = "";
        count = 0;
        totalMesLength = 0;
        isFindEnd = false;
    }
}

void ComManager::clearBuffer()
{
    // if (com.available())
    //     com.flush();
    string rubbish;
    int buffer = 0;
    bool isClear = false;
    while (!isClear)
    {
        buffer = com.available();
        if (buffer)
        {
            rubbish = com.read(buffer);
        }
        else
        {
            isClear = true;
        }
    }
}