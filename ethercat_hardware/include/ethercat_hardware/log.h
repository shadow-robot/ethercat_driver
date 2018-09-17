/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Shadow Robot Company Ltd.
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
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef __ROS_H__
#define __ROS_H__

#include <stdio.h>

#define LOG_DEBUG     4
#define LOG_INFO      3
#define LOG_WARNING   2
#define LOG_ERROR     1
#define LOG_FATAL     0
#define LOG_LEVEL     3

#define ROS_DEBUG(msg, ...) \
  if (LOG_DEBUG <= LOG_LEVEL) fprintf(stderr, (std::string(msg) + "\n").c_str(), ## __VA_ARGS__)
#define ROS_INFO(msg, ...) \
  if (LOG_INFO <= LOG_LEVEL) fprintf(stderr, (std::string(msg) + "\n").c_str(), ## __VA_ARGS__)
#define ROS_WARN(msg, ...) \
  if (LOG_WARNING <= LOG_LEVEL) fprintf(stderr, (std::string(msg) + "\n").c_str(), ## __VA_ARGS__)
#define ROS_ERROR(msg, ...) \
  if (LOG_ERROR <= LOG_LEVEL) fprintf(stderr, (std::string(msg) + "\n").c_str(), ## __VA_ARGS__)
#define ROS_FATAL(msg, ...) \
  if (LOG_FATAL <= LOG_LEVEL) fprintf(stderr, (std::string(msg) + "\n").c_str(), ## __VA_ARGS__)

#endif // __ROS_H__