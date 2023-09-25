#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include "libobsensor/ObSensor.hpp"
#include "opencv2/opencv.hpp"
#include "libobsensor/conio.h"
#define KEY_ESC 27

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

// 保存深度图为png格式
void saveDepth(std::shared_ptr<ob::DepthFrame> depthFrame) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string depthName = "Depth_" + std::to_string(depthFrame->timeStamp()) + ".png";
    cv::Mat     depthMat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data());
    cv::imwrite(depthName, depthMat, compression_params);
    std::cout << "Depth saved:" << depthName << std::endl;
}

// 保存彩色图为png格式
void saveColor(std::shared_ptr<ob::ColorFrame> colorFrame) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string colorName = "Color_" + std::to_string(colorFrame->timeStamp()) + ".png";
    cv::Mat     colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
    cv::imwrite(colorName, colorRawMat, compression_params);
    std::cout << "Color saved:" << colorName << std::endl;
}

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

/*TODO:XXX*/
ob::Pipeline pipeline;
ob::FormatConvertFilter formatConverFilter;
int rgbd_init(orbbec_str ** entity, func_cb rgb, func_cb depth)
{
	int retval = 0;
	*entity = (orbbec_str * )malloc(sizeof(orbbec_str));
	(*entity)->config = std::make_shared<ob::Config>();
	//(*entity)->config = (ob::Config *)malloc(sizeof(ob::Config));

	if ((rgb == NULL) && ((depth == NULL))) {
		std::cerr << "rgb or depth must have callback" << std::endl;
		retval = -1;
	}
	(*entity)->rgb_cb = rgb;
	(*entity)->depth_cb = depth;

	memcpy(&((*entity)->pipeline), &pipeline, sizeof(ob::Pipeline));
	memcpy(&((*entity)->formatConverFilter), &formatConverFilter, sizeof(ob::FormatConvertFilter));
	return retval;
}

/*OB_SENSOR_COLOR/OB_SENSOR_DEPTH*/
int rgbd_func_set(orbbec_str * entity, int which_func, int is_enable)
{
	int retval = 0;
	if (entity != NULL) {
		if (OB_SENSOR_COLOR == which_func) {
			try {
				os_printf("%p\n", &(entity->pipeline));
				auto colorProfiles = (entity->pipeline).getStreamProfileList(OB_SENSOR_COLOR);
				std::shared_ptr<ob::VideoStreamProfile> colorProfile  = nullptr;
				if (colorProfiles) {
					colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(0))->as<ob::VideoStreamProfile>();
				}
				if (is_enable == 1) {
					os_printf("enable color\n");
					entity->config->enableStream(colorProfile);
				} else {
					/*TODO:XXX*/
					os_printf("disable stream\n");
					//entity->config->disableStream(colorProfile);
				}
			}
			catch(ob::Error &e) {
			    std::cerr << "Current device is not support color sensor!" << std::endl;
			    retval = -1;
			}
		} else {
			auto  depthProfiles = entity->pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
			std::shared_ptr<ob::VideoStreamProfile> depthProfile  = nullptr;
			if (depthProfiles) {
				depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(0))->as<ob::VideoStreamProfile>();
			}
			if (is_enable == 1) {
				os_printf("enable depth\n");
				entity->config->enableStream(depthProfile);
			} else {
				/*TODO:XXX*/
				os_printf("disable stream\n");
				//entity->config->disableStream(depthProfile);
			}
		}
	}	
	return retval;

}

int rgb_start(orbbec_str * entity)
{
	int retval = 0;
	if (entity != NULL) {
		retval = rgbd_func_set(entity, OB_SENSOR_COLOR, 1);
	}
	return retval;
}

int depth_start(orbbec_str * entity)
{
	int retval = 0;
	if (entity != NULL) {
		retval = rgbd_func_set(entity, OB_SENSOR_DEPTH, 1);
	}
	return retval;
}

int rgb_stop(orbbec_str * entity)
{
	int retval = 0;
	if (entity != NULL) {
		retval = rgbd_func_set(entity, OB_SENSOR_COLOR, 0);
	}
	return retval;

}
int depth_stop(orbbec_str * entity)
{
	int retval = 0;
	if (entity != NULL) {
		retval = rgbd_func_set(entity, OB_SENSOR_DEPTH, 0);
	}
	return retval;
}

void * rgbd_runtime(void * argv)
{
    	int frameCount = 0;
	orbbec_str * entity = (orbbec_str *)argv;
	os_printf("enter runtime\n");

	entity->pipeline.start(entity->config);

	while (1) {
		auto frameset = entity->pipeline.waitForFrames(100);
        	if(frameset == nullptr) {
        	    std::cout << "The frameset is null!" << std::endl;
        	    continue;
        	}
        	/*strip front frame, because stable picture*/ 
        	if(frameCount < 5) {
        	    frameCount++;
        	    continue;
        	}

        	auto colorFrame = frameset->colorFrame();
        	auto depthFrame = frameset->depthFrame();

		if(colorFrame != nullptr) {
			entity->rgb_count++;
			if (colorFrame->format() == OB_FORMAT_MJPG) {
				entity->formatConverFilter.setFormatConvertType(FORMAT_MJPG_TO_RGB888);
			}
			else if(colorFrame->format() == OB_FORMAT_UYVY) {
				entity->formatConverFilter.setFormatConvertType(FORMAT_UYVY_TO_RGB888);
			}
			else if(colorFrame->format() == OB_FORMAT_YUYV) {
				entity->formatConverFilter.setFormatConvertType(FORMAT_YUYV_TO_RGB888);
			}
			else {
				std::cout << "Color format is not support!" << std::endl;
				continue;
			}
			colorFrame = entity->formatConverFilter.process(colorFrame)->as<ob::ColorFrame>();
			entity->formatConverFilter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
			colorFrame = entity->formatConverFilter.process(colorFrame)->as<ob::ColorFrame>();
			if (entity->rgb_cb != NULL)
				entity->rgb_cb(&colorFrame);
		}

		if(depthFrame != nullptr) {
			entity->depth_count++;
			if (entity->depth_cb != NULL)
				entity->depth_cb(&depthFrame);
			saveDepth(depthFrame);
		}
	}
	/*no reach here*/
}

int rgbd_deinit(orbbec_str * entity)
{
	int retval = 0;
	free(entity);
	return retval;
}

int orbber_run(orbbec_str * entity)
{
	int retval = 0;
	pthread_t task;
	retval = pthread_create(&task, NULL, rgbd_runtime, entity);
	return retval;
}

void * rgb_cb(void * argv)
{
	os_printf("color picture handle\n");
	std::shared_ptr<ob::ColorFrame> colorFrame = *(std::shared_ptr<ob::ColorFrame> *) argv;
	saveColor(colorFrame);
	return NULL;
}

void * depth_cb(void * argv)
{
	os_printf("depth picture handle\n");
	std::shared_ptr<ob::DepthFrame> depthFrame = *(std::shared_ptr<ob::DepthFrame> *)argv;
	saveDepth(depthFrame);
	return NULL;
}

int main()
{
	orbbec_str * entity;
	rgbd_init(&entity, rgb_cb, depth_cb);
	os_printf("%p\n", &(entity->pipeline));
	rgb_start(entity);
	depth_start(entity);
	orbber_run(entity);
	while (1){
		sleep(10);
	};
	rgb_stop(entity);
	depth_stop(entity);
	rgbd_deinit(entity);
}
