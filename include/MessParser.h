#ifndef GNSS_NMEA_PARSER_H
#define GNSS_NMEA_PARSER_H

#include <cstdio>
#include <iostream>
#include <sstream>
#include <bitset>
#include <vector>
#include <map>
#include <queue>

#include <string>
#include <string.h>
#include "Utilities/utilities_print.h"



#define GNSS_NMEA_LINE_SEP "\r\n"
#define GNSS_NMEA_ELEMENT_SEP ","
 
 
//--------------------- 辅助工具----------------------
template <class T>
T stringToNumber(const std::string &sstr)
{
    T number{};
    std::istringstream iss{};
    iss.str(sstr);
    iss >> number; /* can auto remove leading 0 */
    return number;
}

template <class T>
std::string toString(T &value)
{
    std::ostringstream oss{};
    oss << value;
    return oss.str();
}

template <int BS_MAX_SIZE>
void bitsFlagSet(std::bitset<BS_MAX_SIZE> &bs, size_t pos)
{
    bs.set(pos);
}

template <int BS_MAX_SIZE>
void bitsFlagReset(std::bitset<BS_MAX_SIZE> &bs, size_t pos)
{
    bs.reset(pos);
}

template <int BS_MAX_SIZE>
void bitsFlagClear(std::bitset<BS_MAX_SIZE> &bs)
{
    bs.reset();
}

template <int BS_MAX_SIZE>
bool bitsFlagTest(std::bitset<BS_MAX_SIZE> bs, size_t pos)
{
    return bs.test(pos);
}

template <int BS_MAX_SIZE>
std::string bitsFlagToString(std::bitset<BS_MAX_SIZE> &bs)
{
    return bs.to_string();
}

class MessParser
{
public:
    MessParser();
    ~MessParser();
    bool parse(const std::string &mess);
    int getNmeaLines(std::vector<std::string> &lines);
  

    vector<string> headerList;
    // // 初始化原点用了 Reset 函数,默认在(0,0,0)也就是地心
    // void GPS2xyzReset(const sensor_msgs::NavSatFix &msg)
    // {
    //     double latitude = msg.latitude;
    //     double longitude = msg.longitude;
    //     double altitude = msg.altitude;
    //     geo_converter.Reset(latitude, longitude, altitude);
    // }
    // // 功能就是把椭球体下的地理坐标系坐标转为ENU局部系下的坐标
    // geometry_msgs::PointStamped GPS2xyz(const sensor_msgs::NavSatFix &msg)
    // {
    //     double latitude = msg.latitude;
    //     double longitude = msg.longitude;
    //     double altitude = msg.altitude;
    //     // printf("[%lf, %lf]\n", latitude, longtitude);

    //     // latitude = ddmm2dddd(latitude);
    //     // longtitude = ddmm2dddd(longtitude);
    //     // printf("[%.8lf, %.8lf, %.8f]\n", latitude, longitude, altitude);
    //     double xyz[3] = {0, 0, 0};
    //     geo_converter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]); // 将经纬度转化成xyz
    //     // printf("[%lf, %lf, %lf]\n\n", xyz[0], xyz[1], xyz[2]);

    //     geometry_msgs::PointStamped local_pos_msg; // 代表了一个带有参考坐标系和时间戳的点。
    //     local_pos_msg.header.stamp = ros::Time::now();
    //     // local_pos_msg.header.seq = pose_seq++;
    //     local_pos_msg.header.frame_id = "gnss_local_pos";
    //     local_pos_msg.point.x = xyz[0];
    //     local_pos_msg.point.y = xyz[1];
    //     local_pos_msg.point.z = xyz[2];
    //     return local_pos_msg;
    // }
    //     std_msgs::String getNemaMsg()

private:
 
 

    int HexToInt(char _input)
    {
        if (_input >= '0' && _input <= '9')
            return _input - '0';
        else if (_input >= 'a' && _input <= 'f')
            return _input - 'a' + 10;
        else if (_input >= 'A' && _input <= 'F')
            return _input - 'A' + 10;
        else
            return -1;
    }

    int GetCheckSum(std::string _input)
    {
        return HexToInt(_input[0]) * 16 + HexToInt(_input[1]);
    }

   
    bool startsWith(const std::string &src, const std::string &str);
    bool endsWith(const std::string &src, const std::string &str);
    std::string replace(const std::string &raw, const std::string &oldstr, const std::string &newstr);
    size_t split(const std::string &line, const std::string &delim, std::vector<std::string> &vstr);
    void reset();
 std::vector<std::string> m_messElementList;
    std::vector<std::string> m_messLines;
    std::string m_messLineSep = GNSS_NMEA_LINE_SEP;       //"\n"
    std::string m_messElementSep = GNSS_NMEA_ELEMENT_SEP; //","
 
    // GnssStatusValue m_gnssStatus;
};

#endif // GNSS_NMEAPARSER_H
