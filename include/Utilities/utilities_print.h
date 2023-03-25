/**
 * @Author: Wanyue Li
 * @Date: 2023/3/25 14:14:09
 * @LastEditors: Wanyue Li
 * @LastEditTime: 2023/3/25 14:14:09
 * Description: 
 * Copyright: Copyright (©)}) 2023 Wanyue Li. All rights reserved.
 */
 
#ifndef PRINT_OUT_H
#define PRINT_OUT_H
/*!
 * @file Utilities_print.h
 * @brief Common utilities for printing
 */

#include <stdarg.h>
#include <stdio.h>
#include <iostream>
#include <string>

using namespace std;

enum class PrintColor { Default, Red, Green, Yellow, Blue, Magenta, Cyan };

static void printf_color(PrintColor color, const char *fmt, ...)
{
  auto color_id = (uint32_t)color;
  if (color_id)
    printf("\033[1;%dm", (uint32_t)color + 30);
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  printf("\033[0m");
}

/*!
 * fprintf, but with color (used to print color to STDERR)
 */
static void fprintf_color(PrintColor color, FILE *stream, const char *fmt, ...)
{
  auto color_id = (uint32_t)color;
  if (color_id)
    fprintf(stream, "\033[1;%dm", (uint32_t)color + 30);
  va_list args;
  va_start(args, fmt);
  vfprintf(stream, fmt, args);
  va_end(args);
  fprintf(stream, "\033[0m");
}

static void printCharArray(string name, const char *charArr, int len)
{
  cout<<name;
  for (int j = 0; j < len; j++)
  {
    printf("%c", charArr[j]);
  }
  printf("\n");
}

 
#endif
