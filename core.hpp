#pragma once

#ifndef CORE_HPP
#define CORE_HPP
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include "libobsensor/ObSensor.hpp"
#include "opencv2/opencv.hpp"

/* debug printf*/
#define DEBUG 1
#if DEBUG
#define os_printf(format, ...) \
	{printf("[%s : %s : %d] ", \
	__FILE__, __func__, __LINE__); \
	printf(format, ##__VA_ARGS__);}
#else
#define os_printf(format, ...) 
#endif

typedef void *(*func_cb)(void * argv);

typedef struct orbbec_str {
	ob::Pipeline pipeline;
	std::shared_ptr<ob::Config> config;
	//ob::Config * config;
	bool rgb_flag;
	bool depth_flag;
	func_cb rgb_cb;
	func_cb depth_cb;
	double rgb_count;
	double depth_count;
	ob::FormatConvertFilter formatConverFilter;
} orbbec_str;

void saveColor(std::shared_ptr<ob::ColorFrame> colorFrame);
void saveDepth(std::shared_ptr<ob::DepthFrame> depthFrame);
int rgbd_init(orbbec_str ** entity, func_cb rgb, func_cb depth);
int rgb_start(orbbec_str * entity);
int depth_start(orbbec_str * entity);
int rgbd_init(orbbec_str ** entity, func_cb rgb, func_cb depth);
int depth_stop(orbbec_str * entity);
int rgbd_deinit(orbbec_str * entity);
int orbber_run(orbbec_str * entity);
int rgb_stop(orbbec_str * entity);
#endif
