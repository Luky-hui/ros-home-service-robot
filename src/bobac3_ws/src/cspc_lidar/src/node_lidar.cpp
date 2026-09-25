#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <thread>

#include "node_lidar.h"
#include "msg_recept.h"
#include "serial_port.h"
#include "lidar_data_processing.h"
#include "point_cloud_optimize.h"
#include "lidar_information.h"
#include "mtime.h"


using namespace std;

node_lidar_t node_lidar;

//#define ENCRYPTION_T

using std::placeholders::_1;
/*
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("cspc_lidar")
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
      "lidar_status", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::UInt16::SharedPtr msg) const
    {
		switch (msg->data)
		{
			//启动雷达
			case 1:
				node_lidar.lidar_status.lidar_ready = true;
				node_lidar.lidar_status.lidar_abnormal_state = 0;
				printf("#start lidar\n");
				break;
			//关闭雷达
			case 2:
				node_lidar.lidar_status.lidar_ready = false;
				node_lidar.lidar_status.close_lidar = true;
				node_lidar.serial_port->write_data(end_lidar,4);
				printf("#stop lidar\n");
				break;
			//high_exposure
			case 3:
				node_lidar.serial_port->write_data(high_exposure,4);
				break;
			//low_exposure
			case 4:
				node_lidar.serial_port->write_data(low_exposure,4);
				break;
			//清空异常状态
			case 5:
				node_lidar.lidar_status.lidar_abnormal_state = 0;
				break;
			
			default:
				break;
		}
     // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
};
*/
node_lidar_t::~node_lidar_t(){
  	if(scan_node_buf)
	{
		delete[] scan_node_buf;
		scan_node_buf = NULL;
	}
	if (globalRecvBuffer)
	{
		delete[] globalRecvBuffer;
		globalRecvBuffer = NULL;
	}
	printf("关闭雷达");
	node_lidar.serial_port->close();
	node_lidar.lidar_status.lidar_ready = false;
	node_lidar.lidar_status.close_lidar = true;
	flushSerial();

}

/*清空缓存区数据*/
void flushSerial()
{
	if (!node_lidar.lidar_status.isConnected){
		return;
	}
	
	size_t len = node_lidar.serial_port->available();
	
	if (len)
	{
		uint8_t *buffer = static_cast<uint8_t *>(alloca(len * sizeof(uint8_t)));
		size_t bytes_read = node_lidar.serial_port->read_data(buffer, len);
	}

	sleep_ms(20);
}

/*激光雷达启动状态判断函数*/
bool lidar_state_judgment()
{
	static bool status_judge = false;     //整体状态判断
	static bool lidar_flush = false;      //是否已经下发雷达启动指令
	static bool wait_speed_right = false; //是否获取到调速信息
	static bool lidar_start_flag = false; //下发雷达启动指令后的雷达反馈标志

	static uint64_t lidar_status_time = 0; //收到启动指令或者重启指令的时间


	if(node_lidar.lidar_status.lidar_ready != node_lidar.lidar_status.lidar_last_status || node_lidar.lidar_status.close_lidar)
	{
		printf("状态切换\n");

		node_lidar.lidar_status.close_lidar = false;
		node_lidar.lidar_status.lidar_last_status = node_lidar.lidar_status.lidar_ready;

		lidar_flush = false;
		wait_speed_right = false;
		lidar_start_flag = false;

		lidar_status_time = current_times();
		flushSerial();
	}
	if(node_lidar.lidar_status.lidar_trap_restart)
	{
		printf("状态异常重新启动 %lld\n",lidar_status_time);
		
		node_lidar.lidar_status.lidar_trap_restart = false;

		
		wait_speed_right = false;
		lidar_flush = false;
		lidar_start_flag = false;

		lidar_status_time = current_times();
		node_lidar.serial_port->write_data(end_lidar,4);
	}
	if(node_lidar.lidar_status.lidar_ready && !wait_speed_right)
	{
		
		if(current_times() - lidar_status_time > 1000 && !lidar_flush)
		{
			switch (node_lidar.lidar_general_info.version)
			{
				case M1C1_Mini_v1:
					printf("V1版本启动雷达\n");
					node_lidar.serial_port->write_data(start_lidar,4);
					wait_speed_right = true;
					break;
				
				case M1C1_Mini_v2:
					printf("V2 X2版本启动雷达\n");
					node_lidar.serial_port->write_data(start_lidar,4);
					wait_speed_right = true;
					break;
				
				case M1CT_TOF:
					printf("TOF雷达启动\n");
					node_lidar.serial_port->write_data(start_lidar,4);
					wait_speed_right = true;
					break;
				
				default:
					break;
			}
		}
		node_lidar.lidar_time.lidar_frequence_abnormal_time = current_times();
		node_lidar.lidar_time.system_start_time = current_times();
	}
	return wait_speed_right;
}

/************************************************************************/
/*  激光数据解析线程　Laser data analysis thread                           */
/************************************************************************/
int read_forever()
{	
	node_info  local_buf[128];
	size_t     count = 128;
	node_info  local_scan[1000];
	size_t     scan_count = 0;
	result_t   ans = RESULT_FAIL;
	
	memset(local_scan, 0, sizeof(local_scan));

	node_lidar.lidar_time.scan_time_record = current_times();

	while (1)
	{
		bool state_jugde = lidar_state_judgment();
		if(state_jugde)
		{
			count = 128;
			ans = node_lidar.lidar_data_processing.waitScanData(local_buf, count);
			if(!IS_OK(ans))
			{
				if(current_times()-node_lidar.lidar_time.system_start_time > 3000 )
				{
					if(!node_lidar.lidar_status.lidar_restart_try)
					{
						printf("尝试重启雷达\n");
						node_lidar.lidar_status.lidar_restart_try = true;
						node_lidar.lidar_status.lidar_trap_restart = true;
					}else{
						printf("@@@雷达被卡住\n");
						node_lidar.lidar_status.lidar_abnormal_state |= 0x01;
						usleep(100);
					}
				}	
			}else{
				node_lidar.lidar_status.lidar_restart_try = false;
				node_lidar.lidar_time.system_start_time = current_times();
			}
			for (size_t pos = 0; pos < count; ++pos)
			{
				if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)
				{
					if ((local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT))
					{
						local_scan[0].stamp = local_buf[pos].stamp;
						local_scan[0].scan_frequence = local_buf[pos].scan_frequence;
		
						/*频率异常超过30秒，触发异常状态*/
						if(local_scan[0].scan_frequence > 200 || local_scan[0].scan_frequence < 10)
						{
							if(current_times()-node_lidar.lidar_time.lidar_frequence_abnormal_time > 30000)
							{
								node_lidar.lidar_status.lidar_abnormal_state |= 0x02;
							}
						}else{
							node_lidar.lidar_time.lidar_frequence_abnormal_time = current_times();
						}
						
						
						node_lidar._lock.lock();
						if((node_lidar.lidar_time.scan_time_current - node_lidar.lidar_time.scan_time_record) > 2000)
						{
							//printf("full----- count=%d,time=%lld\n",scan_count,current_times());
							node_lidar.lidar_time.scan_time_record = node_lidar.lidar_time.scan_time_current;
						}
						memcpy(node_lidar.scan_node_buf, local_scan, scan_count * sizeof(node_info));
						node_lidar.scan_node_count = scan_count;
						node_lidar.lidar_time.scan_time_current = current_times();
						node_lidar._dataEvent.set();
						node_lidar._lock.unlock();
					}
					scan_count = 0;
				}
				local_scan[scan_count++] = local_buf[pos];
				if (scan_count == _countof(local_scan))
				{
					scan_count -= 1;
				}
			}
		}
		else{
			flushSerial();
			delay(100);
		}
		
	}
	return RESULT_OK;
}

/*线程事件同步函数*/
result_t grabScanData(uint32_t timeout) {
	switch (node_lidar._dataEvent.wait(timeout)) {
		case Event::EVENT_TIMEOUT:
			return RESULT_TIMEOUT;

		case Event::EVENT_OK: 
			{
				node_lidar._lock.lock();
				if (node_lidar.scan_node_count == 0) {
					return RESULT_FAIL;
				}
				node_lidar._lock.unlock();
			}
			return RESULT_OK;

		default:
			return RESULT_FAIL;
	}
}

/*处理雷达的线程*/
bool data_handling(LaserScan &outscan)
{

	//node_lidar.lidar_time.tim_scan_start = getTime();	
	if(grabScanData(2000)==RESULT_OK)
	{
		send_lidar_data(outscan);
		return true;
	}else{
		return false;
	}
	
}


/*处理最新一圈雷达的数据*/
int send_lidar_data(LaserScan &outscan)
{
	node_lidar._lock.lock();

	size_t count = node_lidar.scan_node_count;

	if(count < MAX_SCAN_NODES && count > 0)
	{	
		node_lidar.lidar_time.tim_scan_end = getTime();
		uint64_t scan_time = node_lidar.lidar_general_info.m_PointTime * (count - 1);
		node_lidar.lidar_time.tim_scan_start = node_lidar.lidar_time.tim_scan_end -  scan_time ;

		node_lidar.lidar_block.lidar_zero_count = 0;


		outscan.config.angle_increment = (2.0*M_PI/count);
		outscan.config.min_angle = 0;
		outscan.config.max_angle = 2*M_PI;
		outscan.config.min_range = 0.10;
		outscan.config.max_range = 10.0; //测量的最远距离是10m
		outscan.config.scan_time =  static_cast<float>(scan_time * 1.0 / 1e9);
    	outscan.config.time_increment = outscan.config.scan_time / (double)(count - 1);
		outscan.stamp = node_lidar.lidar_time.tim_scan_start;
		//scan_msg->header.frame_id = node_lidar.lidar_general_info.frame_id;
		//scan_msg->header.stamp = ros::Time::now();

		if(node_lidar.lidar_status.isConnected)
		{
			//outscan.points.clear();
			float range = 0;
			float angle = 0.0;
			uint16_t intensity = 0;
			for (int i = 0; i < count; i++)
			{
				LaserPoint point;
				LaserPoint point_check;
				angle = static_cast<float>((node_lidar.scan_node_buf[i].angle_q6_checkbit >>
											LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /64.0f);
				range = (node_lidar.scan_node_buf[i].distance_q2/1000.0f);
				
				intensity = node_lidar.scan_node_buf[i].sync_quality;

				point_check.angle = angle;
				point_check.range = range;
				point_check.intensity = intensity;
				
				if(0 <= angle && angle <= 360){
					point.range = range;
					point.angle = angle;
					point.intensity = intensity;
				}else{
					point.range = 0.0;
					point.intensity = 0;
					point.angle = 0.0;
				}

				if(range < 0.1)
				{
					node_lidar.lidar_block.lidar_zero_count++;
				}
				
				outscan.points.push_back(point);
			}

			node_lidar._lock.unlock();

			/*雷达被遮挡判断*/
			
			node_lidar.optimize_lidar.lidar_blocked_judge(count);

            /*
			if (node_lidar.lidar_status.FilterEnable)
			{
				node_lidar.optimize_lidar.PointCloudFilter(&outscan);
			}
            */

		}
	}
	node_lidar._lock.unlock();
}

/*设置串口信息的函数*/
bool lidar_set_port()
{
	if (node_lidar.lidar_status.isConnected)
	{
		return true;
	}

	std::cout << "***" << node_lidar.lidar_general_info.port << std::endl;
	node_lidar.serial_port=make_shared<Serial_Port>(node_lidar.lidar_general_info.port,
							node_lidar.lidar_general_info.m_SerialBaudrate,Timeout::simpleTimeout(DEFAULT_TIMEOUT));
	if (!node_lidar.serial_port->open())
	{
		return false;
	}
	node_lidar.lidar_status.isConnected=true;
	sleep_ms(100);
    node_lidar.serial_port->setDTR(0);
	return true;
}

/*初始化函数*/
bool initialize()
{
	if(node_lidar.lidar_status.optimize_enable)
	{
		//求雷达安装位置到扫地机边缘的距离
		node_lidar.optimize_lidar.lidar_blocked_init();
	}

	switch (node_lidar.lidar_general_info.version)
	{
	case M1C1_Mini_v1:
		printf("version M1C1_Mini_v1\n");
		node_lidar.lidar_general_info.m_SerialBaudrate = 115200;
		node_lidar.lidar_data_processing.PackageSampleBytes = 2;
		break;

	case M1C1_Mini_v2:
		printf("version M1C1_Mini_v2\n");
		node_lidar.lidar_general_info.m_SerialBaudrate = 150000;
		node_lidar.lidar_data_processing.PackageSampleBytes = 3;
		node_lidar.lidar_general_info.m_intensities = true;
		break;

	case M1CT_Coin_Plus:
		printf("version M1CT_Coin_Plus\n");
		node_lidar.lidar_general_info.m_SerialBaudrate = 115200;
		node_lidar.lidar_data_processing.PackageSampleBytes = 3;
		break;
	
	case M1CT_TOF:
		printf("version M1CT_TOF\n");
		node_lidar.lidar_general_info.m_SerialBaudrate = 230400;
		node_lidar.lidar_data_processing.PackageSampleBytes = 3;
		node_lidar.lidar_general_info.m_intensities = true;
		break;
	
	default:
		break;
	}

	//设置通信串口
	if(!lidar_set_port()){
		printf("lidar_set_port wrong\n");
		return false;
	}

	//获取串口获取每个byte所用的时间
	node_lidar.lidar_general_info.trans_delay = node_lidar.serial_port->getByteTime();
	node_lidar.scan_node_buf = new node_info[1000];
	node_lidar.globalRecvBuffer = new uint8_t[sizeof(node_packages)];
	return true;
}


#ifdef ROS_2
int topic_thread()
{
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
}
#endif

int node_start(int argc, char **argv)
{

	ros::init(argc, argv, "M1CT_Coin_Plus");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	double cut_angle_min,cut_angle_max;

	priv_nh.param("baud_rate", node_lidar.lidar_general_info.m_SerialBaudrate, 115200);
	priv_nh.param("frame_id", node_lidar.lidar_general_info.frame_id, std::string("base_link"));
	priv_nh.param("port", node_lidar.lidar_general_info.port, std::string("/dev/sc_mini"));
	priv_nh.param("version", node_lidar.lidar_general_info.version, 2);
	priv_nh.param("angle_min", node_lidar.lidar_general_info.angle_min, 0.0);
	priv_nh.param("angle_max", node_lidar.lidar_general_info.angle_max, 360.0);
	priv_nh.param("cut_angle_min", cut_angle_min, 0.0);
	priv_nh.param("cut_angle_max", cut_angle_max, 0.0);


	// std::cout<<"====================="<<cut_list<<std::endl;
	ROS_INFO("angle_min : %f",node_lidar.lidar_general_info.angle_min);
	ROS_INFO("angle_max : %f",node_lidar.lidar_general_info.angle_max);

	/*
	auto subscription_ = node->create_subscription<std_msgs::msg::UInt16>(
      "lidar_status", 10, topic_callback);*/
	
	
	bool ret_init = initialize();

	if(!ret_init){
		printf("node_lidar init error\n");
		return -1;
	}
	/*读取激光雷达数据的线程*/
	thread t1(read_forever);
	t1.detach();


	/*处理雷达数据的线程*/
	/*
	thread t2(data_handling(node));
	t2.detach();*/
	
	node_lidar.lidar_status.lidar_ready = true;
	ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

/*ros2为rclcpp::ok()*/
	while(ros::ok())
	{
#ifdef ROS_2
		if(node_lidar.lidar_status.lidar_abnormal_state != 0)
		{
			if(node_lidar.lidar_status.lidar_abnormal_state & 0x01)
			{
				pubdata.data="node_lidar is trapped\n";
				error_pub->publish(pubdata);
				printf("异常状态1---trapped\n");
			}
			if(node_lidar.lidar_status.lidar_abnormal_state & 0x02)
			{
				pubdata.data="node_lidar frequence abnormal\n";
				error_pub->publish(pubdata);
				printf("异常状态2---frequence abnormal\n");
			}
			if(node_lidar.lidar_status.lidar_abnormal_state & 0x04)
			{
				pubdata.data="node_lidar is blocked\n";
				error_pub->publish(pubdata);
				printf("异常状态3---blocked\n");
			}
			node_lidar.serial_port->write_data(end_lidar,4);
			node_lidar.lidar_status.lidar_ready = false;

			sleep(1);
		}
#endif
		LaserScan scan;
		double min_increment,max_increment;
		if(data_handling(scan))
		{
			sensor_msgs::LaserScan scan_pub;
			// scan_pub.ranges.resize(scan.points.size());
			// scan_pub.intensities.resize(scan.points.size());
			scan_pub.angle_increment = (2.0*M_PI/scan.points.size());
			scan_pub.angle_min = M_PI/180.0*node_lidar.lidar_general_info.angle_min;
			scan_pub.angle_max = M_PI/180.0*node_lidar.lidar_general_info.angle_max;

			scan_pub.range_min = 0.10;
			scan_pub.range_max = 10.0; //测量的最远距离是10m
			scan_pub.header.frame_id = node_lidar.lidar_general_info.frame_id;
			scan_pub.header.stamp = ros::Time::now();
			scan_pub.ranges.clear();
			scan_pub.intensities.clear();
			min_increment = cut_angle_min/360.0*scan.points.size();
			max_increment = cut_angle_max/360.0*scan.points.size();
			for(int i=min_increment;i<max_increment;i++){
				scan.points[i].range = 0.0;
				scan.points[i].range = 0.0;
			}
			min_increment = node_lidar.lidar_general_info.angle_min/360.0*scan.points.size();
			max_increment = node_lidar.lidar_general_info.angle_max/360.0*scan.points.size();
			for(int i=min_increment; i < max_increment; i++) {
				scan_pub.ranges.push_back(scan.points[i].range);
				scan_pub.intensities.push_back(scan.points[i].range);
			}
			laser_pub.publish(scan_pub);
      	}
	}
	node_lidar.serial_port->write_data(end_lidar,4);
	return 0;
}

