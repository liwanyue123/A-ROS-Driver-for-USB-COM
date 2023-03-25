/**
 * @Author: Wanyue Li
 * @Date: 2023/3/25 14:14:01
 * @LastEditors: Wanyue Li
 * @LastEditTime: 2023/3/25 14:14:01
 * Description: 
 * Copyright: Copyright (©)}) 2023 Wanyue Li. All rights reserved.
 */
/*!
 * @file PeriodicTask.h
 * @brief Implementation of a periodic function running in a separate thread.
 * Periodic tasks have a task manager, which measure how long they take to run.
 * 在单独线程中运行的周期函数的实现。周期性任务有一个任务管理器，用于度量它们运行所需的时间。
 */

#ifndef PERIODIC_TASK_H
#define PERIODIC_TASK_H
#ifdef linux
 #include <sys/timerfd.h>
#endif

#include <unistd.h>
#include <cmath>

#include "Timer.h"
#include "Utilities/utilities_print.h"
#include <string>
#include <thread>
#include <vector>

class PeriodicTaskManager;

/*!
 * A single periodic task which will call run() at the given frequency
 * 一个周期任务，它将以给定的频率调用run() run()被定义在继承的子类中
 */
class PeriodicTask {
 public:
  PeriodicTask(PeriodicTaskManager* taskManager, float period,
               std::string name);//构造时会将构造的任务添加到任务管理器 taskManager
  void start(); //开始运行的任务 开一个线程运行执行周期运行程序
  void stop();//停止运行的任务
  void close();
  void printStatus();//打印运行状态
  void clearMax();//清楚最大值 下面的几个max
  bool isSlow();//慢速运行
  //纯虚函数只有函回数的名字而不具备函数的功能，不能被调用。它只是通知编译系统:
  
//  “在这里声明一个虚函数，留待派生类中定义”。在答派生类中对此函数提供定义后，它才能具备函数的功能，可被调用。
  virtual void init() = 0;
  virtual void run() = 0;
  virtual void cleanup() = 0;
  virtual ~PeriodicTask() { stop(); }

  /*!
   * Get the desired period for the task 获得任务所需的周期
   */
  float getPeriod() { return _period; }

  /*!
   * Get how long the most recent run took 了解最近一次跑步花了多长时间
   */
  float getRuntime() { return _lastRuntime; }

  /*!
   * Get the maximum time in between runs 获取运行之间的最大时间
   */ 
  float getMaxPeriod() { return _maxPeriod; }

  /*!
   * Get the maximum time it took for a run 获得运行所需的最大时间
   */
  float getMaxRuntime() { return _maxRuntime; }

 private:
  void loopFunction();//运行代码的地方 以一个固定周期运行

  float _period;//周期
  volatile bool _running = false;
  float _lastRuntime = 0;
  float _lastPeriodTime = 0;
  float _maxPeriod = 0;
  float _maxRuntime = 0;
  std::string _name;//任务名
  std::thread _thread;//任务线程
};

/*!
 * A collection of periodic tasks which can be monitored together
 * 可以一起监视的定期任务的集合
 */
class PeriodicTaskManager {
 public:
  PeriodicTaskManager() = default;
  ~PeriodicTaskManager();
  void addTask(PeriodicTask* task);
  void printStatus();
  void printStatusOfSlowTasks();
  void stopAll();
  void closeAll();
 private:
  std::vector<PeriodicTask*> _tasks;
};

/*!
 * A periodic task for calling a function
 * 用于调用函数的周期性任务
 */
class PeriodicFunction : public PeriodicTask {
 public:
  PeriodicFunction(PeriodicTaskManager* taskManager, float period,
                   std::string name, void (*function)())
      : PeriodicTask(taskManager, period, name), _function(function) {}
  void cleanup() {}
  void init() {}
  void run() { _function(); }

  ~PeriodicFunction() { stop(); }

 private:
  void (*_function)() = nullptr;
};

/*!
 * A periodic task for printing the status of all tasks in the task manager
 * 用于打印任务管理器中所有任务的状态的定期任务
 */
class PrintTaskStatus : public PeriodicTask {
 public:
  PrintTaskStatus(PeriodicTaskManager* tm, float period)
      : PeriodicTask(tm, period, "print-tasks"), _tm(tm) {}
  void run() override { 
    // DH: Disable printing
    //_tm->printStatus(); 
  }

  void init() override {}

  void cleanup() override {}

 private:
  PeriodicTaskManager* _tm;
};

/*!
 * A periodic task for calling a member function
 * 用于调用成员函数的周期性任务
 */
template <typename T>
class PeriodicMemberFunction : public PeriodicTask {
 public:
  PeriodicMemberFunction(PeriodicTaskManager* taskManager, float period,
                         std::string name, void (T::*function)(), T* obj)
      : PeriodicTask(taskManager, period, name),
        _function(function),
        _obj(obj) {}

  void cleanup() {}
  void init() {}
  void run() { (_obj->*_function)(); }

 private:
  void (T::*_function)();
  T* _obj;
};

#endif  // PERIODIC_TASK_H
