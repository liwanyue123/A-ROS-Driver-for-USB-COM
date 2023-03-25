/**
 * @Author: Wanyue Li
 * @Date: 2023/3/25 14:14:06
 * @LastEditors: Wanyue Li
 * @LastEditTime: 2023/3/25 14:14:06
 * Description: 
 * Copyright: Copyright (©)}) 2023 Wanyue Li. All rights reserved.
 */
// /*!
//  * @file utilities.h
//  * @brief Common utility functions
//  */

#ifndef PROJECT_UTILITIES_H
#define PROJECT_UTILITIES_H

#include <algorithm>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <string>
#include <string.h>
#include <sstream>

#include <iomanip>
#include <fstream>
// file
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#include <ros/ros.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <unistd.h>

using namespace std;

// // #include "../cppTypes.h"

namespace NumUtils
{

    /*!
     * Are two floating point values almost equal?
     判断两个浮点数是不是几乎相等
     * @param a : first value
     * @param b : second value
     * @param tol : equality tolerance
     */
    template <typename T>
    bool fpEqual(T a, T b, T tol)
    {
        return std::abs(a - b) <= tol;
    }

    /*!
     * Are two std::vectors equal?
      判断两个vector是不是相等
     * Compares with "!=" operator
     */
    template <typename T>
    bool vectorEqual(const std::vector<T> &a, const std::vector<T> &b)
    {
        if (a.size() != b.size())
            return false;
        for (int i = 0; i < a.size(); i++)
        {
            if (a[i] != b[i])
                return false;
        }
        return true;
    }

    /*!
     * Coerce in to be between min and max
     将一个数限制在min和max之中
     */
    template <typename T>
    T coerce(T in, T min, T max)
    {
        if (in < min)
        {
            in = min;
        }
        if (in > max)
        {
            in = max;
        }
        return in;
    }

    /*!
     * Apply deadband
     在死区（-range，range）之中的数全部变成0
     * @param x : input
     * @param range : deadband (+/- range around 0)
     * @return result
     */
    template <typename T>
    T deadband(T x, T range)
    {
        if (x < range && x > -range)
            x = T(0);
        return x;
    }

    // /*!
    //  * Apply deadband to eigen type
    //  */
    // template <typename T>
    // void eigenDeadband(Eigen::MatrixBase<T>& v, typename T::Scalar band) {
    //   for (int i = 0; i < T::RowsAtCompileTime; i++) {
    //     for (int j = 0; j < T::ColsAtCompileTime; j++) {
    //       v(i, j) = deadband(v(i, j), band);
    //     }
    //   }
    // }

    /*!
     * Get the sign of a number
     获取一个数字是正负还是0,1正，-1负，0就是0
     * 1 for positive, 0 for 0, -1 for negative...
     */
    template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    // /*!
    // 生产随机数填充eigen
    //  * Fill an eigen type with random numbers from a random generator and uniform
    //  * real distribution.
    //  * TODO: is there a way to make this work nicely with normal distributions too?
    //  */
    // template <typename T>
    // void fillEigenWithRandom(
    //     Eigen::MatrixBase<T> &v, std::mt19937 &gen,
    //     std::uniform_real_distribution<typename T::Scalar> &dist)
    // {
    //     for (int i = 0; i < T::RowsAtCompileTime; i++)
    //     {
    //         for (int j = 0; j < T::ColsAtCompileTime; j++)
    //         {
    //             v(i, j) = dist(gen);
    //         }
    //     }
    // }

    /*!
     * Generate a random number following normal distribution
     生成一个服从正太分布的随机数
     */
    template <typename T>
    T generator_gaussian_noise(T mean, T var)
    {
        static bool hasSpare = false;
        static T rand1, rand2;

        if (hasSpare)
        {
            hasSpare = false;
            return mean + sqrt(var * rand1) * sin(rand2);
        }
        hasSpare = true;

        rand1 = rand() / ((T)RAND_MAX);
        if (rand1 < 1e-100)
            rand1 = 1e-100;
        rand1 = -2 * log(rand1);
        rand2 = rand() / ((T)RAND_MAX) * M_PI * 2.;

        // printf("rand: %f, %f\n", rand1, rand2);
        return mean + sqrt(var * rand1) * cos(rand2);
    }
}

namespace PrintUtils
{
    void printCharArray(char *arr, int len)
    {
        for (int i = 0; i < len; i++)
        {
            std::cout << arr[i] << " ";
        }
        std::cout << std::endl;
    }
    void printUnCharArray(unsigned char *arr, int len)
    {
        for (int i = 0; i < len; i++)
        {
            // std::cout << arr[i] << " ";
            std::cout << arr[i];
        }
        std::cout << std::endl;
    }
    void printUint8Array(const uint8_t *arr, int len)
    {
        for (int i = 0; i < len; i++)
        {
            // std::cout << arr[i] << " ";
            std::cout << arr[i];
        }
        std::cout << std::endl;
    }
}

namespace TranUtils
{
    /*!
    数字转string
     * Convert a floating point number to a string.  Is preferable over
     * std::to_string because this uses scientific notation and won't truncate
     * small/large numbers.
     */
    template <typename T>
    std::string numberToString(T Number)
    {
        std::ostringstream ss;
        ss << Number;
        return ss.str();
    }
    /*!
    线性的映射，将 (inputMin, inputMax)范围里的数，投影到(outputMin, outputMax)上
     * map value x in (inputMin, inputMax) to (outputMin, outputMax) linearly
     */
    template <typename T>
    T mapToRange(T x, T inputMin, T inputMax, T outputMin, T outputMax)
    {
        return outputMin +
               (x - inputMin) * (outputMax - outputMin) / (inputMax - inputMin);
    }

    // /*!
    //  * Convert eigen type to std::string.
    //  将eigen转成string
    //  */
    // template <typename T>
    // std::string eigenToString(Eigen::MatrixBase<T>& value) {
    //   std::stringstream ss;
    //   ss << value;
    //   return ss.str();
    // }

    /*!
     * Convert boolean to string (true, false)
      将bool转成string
     */
    static inline std::string boolToString(bool b)
    {
        return std::string(b ? "true" : "false");
    }

    /*!
      将string转成char[]
     */
    static inline char *stringToChar(std::string str_source)
    {
        return const_cast<char *>(str_source.c_str());
    }
    /*!
      将char[]转成string
     */
    static inline std::string CharTostring(char *str)
    {
        return std::string(str);
    }
    /*!
    * Convert from string to float or double.
    将字符串string转成数字float or double.
    */
    template <typename T>
    T stringToNumber(const std::string &str)
    {
        static_assert(std::is_same<T, double>::value || std::is_same<T, float>::value,
                      "stringToNumber only works for double/float");

        if (std::is_same<T, double>::value)
        {
            return std::stod(str);
        }
        else if (std::is_same<T, float>::value)
        {
            return std::stof(str);
        }
    }

    // 下面这些写不成类,因为指针的原因
    //  string 转为 unsinged char*

    // void stringToUnChar(string str, unsigned char *ch_ptr)
    // {
    //     unsigned char *ch_ptr = (unsigned char *)str.c_str();
    // }

    string UnCharToString(const unsigned char *foo)
    {
        // string str=reinterpret_cast<const char *>(foo);
        string str;
        str = reinterpret_cast<const char *>(foo);
        return str;
    }

    // 下面这些写不成类
    //  void stringToUint8(string  str, const uint8_t* ch_ptr)
    //  {
    //      const uint8_t* ch_ptr= reinterpret_cast<const uint8_t*>(str.c_str());
    //  }

    string uint8ToString(const uint8_t *foo)
    {
        // string str=reinterpret_cast<const char *>(foo);
        string str;
        str = reinterpret_cast<const char *>(foo);
        return str;
    }

    template <typename T>
    T stringToNumber(std::string str)
    {
        static_assert(std::is_same<T, double>::value || std::is_same<T, float>::value,
                      "stringToNumber only works for double/float");

        if (std::is_same<T, double>::value)
        {
            return std::stod(str);
        }
        else if (std::is_same<T, float>::value)
        {
            return std::stof(str);
        }
    }

    /*!
     * Convert from string to float or double
     将char*转成数字float or double.
     */
    template <typename T>
    T charToNumber(const char *str)
    {
        //   return stringToNumber<T>(std::string(str));
        return atof(str);
    }

    // c_str()将std_msgs::string转换为std::string
}

namespace TimeUtils
{

    /*!
     * Get the current time and date as a string
     获取当前时间
     */
    std::string getCurrentTimeAndDate()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream ss;
        ss << std::put_time(&tm, "%c");
        return ss.str();
    }
    // 以时间作为图片名 精确到毫秒
    std::string GetTimeAsFileName(ros::Time current_time)
    {
        uint32_t sec = current_time.sec;
        uint32_t nsec = current_time.nsec;

        // 将时间戳转换为本地时间
        time_t t = static_cast<time_t>(sec);
        struct tm *tm = localtime(&t);

        // 获取时间信息
        int year = tm->tm_year + 1900;
        int month = tm->tm_mon + 1;
        int day = tm->tm_mday;
        int hour = tm->tm_hour;
        int minute = tm->tm_min;
        int second = tm->tm_sec;
        int millisecond = static_cast<int>(nsec / 1e6);

        // 当前时间 可精确到ms
        char filename[100] = {0};
        // sprintf(filename, "pictures_src/%d%.png", st);
        sprintf(filename, "%d%02d%02d_%02d%02d%02d%03d.png", year, month, day, hour, minute, second, millisecond);
        return filename;
    }

    // 以时间作为图片名 精确到毫秒
    std::string GetTimeAsFileName()
    {

        ros::Time current_time = ros::Time::now(); // 获取当前时间 可精确到ms
        uint32_t sec = current_time.sec;
        uint32_t nsec = current_time.nsec;

        // 将时间戳转换为本地时间
        time_t t = static_cast<time_t>(sec);
        struct tm *tm = localtime(&t);

        // 获取时间信息
        int year = tm->tm_year + 1900;
        int month = tm->tm_mon + 1;
        int day = tm->tm_mday;
        int hour = tm->tm_hour;
        int minute = tm->tm_min;
        int second = tm->tm_sec;
        int millisecond = static_cast<int>(nsec / 1e6);

        // 当前时间 可精确到ms
        char filename[100] = {0};
        // sprintf(filename, "pictures_src/%d%.png", st);
        sprintf(filename, "%d%02d%02d_%02d%02d%02d%03d.png", year, month, day, hour, minute, second, millisecond);
        
        // cout<<filename<<endl;
        return filename;
    }

    // . 时间戳转换为浮点型

    // double time2;
    // time2 = time1.toSec();

    //     1
    //     2

    // 5. 浮点型转换为时间戳

    // msg.header.stamp = time2.fromSec();
}

namespace FileOperator
{
    void getCurPath(char *buffer)
    {
        buffer = getcwd(NULL, 0);
        cout << "文件路径" << buffer << endl;
        // 将需要调用的模块使用 strcat 作拼接;
        // const char *model_path = strcat(buffer, "/models");
    }

    // 检查文件夹是否存在
    bool checkPathExists(string path)
    {
        DIR *dir;
        // int isCreate = 0;

        if ((dir = opendir((path).c_str())) == NULL)
        {
            // isCreate = mkdir((path).c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO); // 后面的是文件夹权限
            return false;
        }
        return true;
        // if (isCreate == 0)
        //     return true;
        // else
        //     return false;
    }
    bool checkFileExists(string &name)
    {
        return (access(name.c_str(), F_OK) != -1);
    }

    bool createDirs(const string &dirName)
    {

        string command_down = "mkdir -p " + dirName;
        system(command_down.c_str());

        return true;
    }
    bool createFile(const string &fullPath)
    {
        // 全路径名
        string fileName = "";
        string pathName = "";

        int endCmpPath = fullPath.size();

        int phit = fullPath.rfind("/", endCmpPath); // delim是",",phit是，在原报文中的位置
        if (std::string::npos != phit)
        {
            /* find delim, get substr */
            fileName = fullPath.substr(phit + 1, endCmpPath); // 切开
            pathName = fullPath.substr(0, phit + 1);          // 切开
        }
        createDirs(pathName);
        string command_down = "touch " + fullPath;
        system(command_down.c_str());
        printf("Create: %s successfully\n", fullPath.c_str());
        return true;
    }

}

namespace StringUtils
{
    // #define COM_NMEA_LINE_SEP "\r\n"
    //  std::string m_messLineSep = COM_NMEA_LINE_SEP;       //"\n"
    // std::vector<std::string> m_messLines;
    //(void)split(mess, m_messLineSep, m_messLines);
    // 根据delim拆分line，存入vstr

    int splitString(const std::string &line, const std::string &delim, std::vector<std::string> &vstr)
    {
        int pstart = 0;
        int phit = 0;
        std::string sstr;
        int length = line.length();

        vstr.clear();
        for (; pstart <= length;)
        {
            phit = line.find(delim, pstart); // delim是",",phit是，在原报文中的位置
            if (std::string::npos != phit)
            {
                /* find delim, get substr */
                sstr = line.substr(pstart, phit - pstart); // 切开
                vstr.push_back(sstr);                      // 存入堆栈
                pstart = phit + delim.size();
            }
            else
            {
                /* not find delim, append remaining str and break */
                vstr.push_back(line.substr(pstart));
                break;
            }
        }
        return vstr.size();
    }


    void continueWriteFile(FILE *fp,
                           const std::string &fileData)
    {

        if (!fp)
        {
            printf("File is not opened \n");
            throw std::runtime_error("Failed to open file");
        }
        else
        {
            fprintf(fp, "%s", fileData.c_str());
        }
        //   fwrite(fileData,sizeof(fileData),1,fp);
    }

    void closeFile(FILE *fp)
    {

        fclose(fp);
    }

    void writeStringToFile(const std::string &fileName,
                           const std::string &fileData)
    {
        FILE *fp = fopen(fileName.c_str(), "w");
        if (!fp)
        {
            printf("Failed to fopen %s\n", fileName.c_str());
            throw std::runtime_error("Failed to open file");
        }
        fprintf(fp, "%s", fileData.c_str());
        fclose(fp);
    }

}

namespace OtherUtils
{


    // Smooth Changing
    /*!
     * Interpolate with cosine (sometimes called coserp)
     余弦插值
     */
    template <typename T>
    T smooth_change(T ini, T end, T moving_duration, T curr_time)
    {
        if (curr_time > moving_duration)
        {
            return end;
        }
        return (ini +
                (end - ini) * 0.5 * (1 - cos(curr_time / moving_duration * M_PI)));
    }

    /*!
     * Derivative of smooth_change
     */
    template <typename T>
    T smooth_change_vel(T ini, T end, T moving_duration, T curr_time)
    {
        if (curr_time > moving_duration)
        {
            return 0.0;
        }
        return ((end - ini) * 0.5 * (M_PI / moving_duration) *
                sin(curr_time / moving_duration * M_PI));
    }

    /*!
     * Derivative of smooth_change_vel
     */
    template <typename T>
    T smooth_change_acc(T ini, T end, T moving_duration, T curr_time)
    {
        if (curr_time > moving_duration)
        {
            return 0.0;
        }
        return ((end - ini) * 0.5 * (M_PI / moving_duration) *
                (M_PI / moving_duration) * cos(curr_time / moving_duration * M_PI));
    }


    /*!
     * Does the unordered map contain the given element?
     map中有这个key吗？
     */
    template <typename T1, typename T2>
    bool uMapContains(const std::unordered_map<T1, T2> &set, T1 key)
    {
        return set.find(key) != set.end();
    }
}

#endif // PROJECT_UTILITIES_H
