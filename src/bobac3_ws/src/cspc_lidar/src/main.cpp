#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include "node_lidar.h"


int parse_args(int argc, char **argv)
{
	struct option opts[] = {
		{"cover", 1, 0, 256}, //剔除雷达上盖结构的缺口
		{"diameter",1,0,257}, //扫地机机身直径
		{"center",1,0,258},   //扫地机中心到雷达中心的距离（雷达处于扫地机前半边数值为正，雷达中心处于扫地机后半边数值为负）
		{"gyr",1,0,259},      //是否在雷达节点做旋转补偿
		{"slam",0,0,260},     //是否在待机状态让雷达节点解析数据
		{"angle",1,0,261},    //雷达的安装角度（顺时针偏转）
		{"version",1,0,262},  //雷达的版本（1:M1C1_Mini_v1;2:M1C1_Mini_V2;3:M1CT_Coin_plus）
		{"baudrate",1,0,263}, //雷达的波特率
		{"encry",0,0,264},   
		{"exp",0,0,265},   
		{0, 0, 0, 0} 
	};
	int ch;
	while ((ch = getopt_long_only(argc, argv, "hcfs", opts, NULL)) != -1)
	{
		switch (ch)
		{
		case 'h':
			printf("lidar node Help:\n");
			printf("./node_lidar /dev/ttyS2 -c -d.\n");
			printf("node lidar -c : no rotate compensate.\n");
			printf("node lidar -f : no lidar data filter.\n");
			printf("node lidar -cover [57,70][101,117][184,196][228,241][291,305][335,350] :note, no blank between []\n");
			exit(0);
			break;
		case 'c':
			break;
		case 'f':
			break;
		case 's':
			break;
		case 256:
			node_lidar.lidar_status.lidar_cover = true;
			node_lidar.optimize_lidar.getLidarCoverAngle(optarg);
			break;
		case 257:
			node_lidar.lidar_robot_info.ROBOT_DIAMETER_mm=atoi(optarg);
			break;
		case 258:
			node_lidar.lidar_robot_info.LIDAR_ROBOT_CENTER_DISTANCE_mm=atoi(optarg);
			break;
		case 259:
			node_lidar.lidar_status.GyroCompensateEnable = true;
			break;
		case 260:
			node_lidar.lidar_status.slam_user = true;
			break;
		case 261:
			node_lidar.lidar_robot_info.install_to_zero = atof(optarg);
			break;
		case 262:
			node_lidar.lidar_general_info.version = atoi(optarg);
			break;
		case 263:
			node_lidar.lidar_general_info.m_SerialBaudrate = atoi(optarg);
			break;
		case 264:
			node_lidar.lidar_status.disable_encry = true;
			break;
		case 265:
			node_lidar.lidar_status.able_exposure = true;
			break;

		default:
			printf("error: unknow arg in ths args\n");
			break;
		}
	}
	return 0;
}


int main(int argc, char **argv)
{
	/*
	if(argc < 2)
	{
		printf("args error\n");
		return 0;
	}*/
	
	fflush(stdout);

	node_start(argc, argv);
}                                                                                                           
