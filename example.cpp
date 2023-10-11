#include "core.hpp"
/*user API*/
void * rgb_cb(void * argv)
{
	os_printf("color picture handle\n");
	std::shared_ptr<ob::ColorFrame> colorFrame = *(std::shared_ptr<ob::ColorFrame> *) argv;
	saveColor(colorFrame);
	return NULL;
}

/*user API*/
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
