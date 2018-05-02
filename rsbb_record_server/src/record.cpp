#include "record.h"
#include <ros/ros.h>
#include <boost/thread.hpp>

Record::Record(ros::NodeHandle &nh, rosbag::RecorderOptions const& options) :
		rosbag::Recorder(options) {
//	service_srv = nh.advertiseService("cmd", &Record::string_command, this);
//	b_record = false;
//	stopped = false;
}

//void Record::wait_for_callback() {
//
//	ros::Rate r(100); // 100 Hz
//
//	while (!b_record && ros::ok()) {
//		ros::spinOnce();
//		r.sleep();
//	}
//
//}

void Record::stop_recording() {

	ros::shutdown();

}

//bool Record::string_command(record_ros::String_cmd::Request& req, record_ros::String_cmd::Response& res) {
//
//	std::string cmd = req.cmd;
//	ROS_INFO("Record callback");
//
//	if (cmd == "record") {
//
//		if (b_record) {
//
//			ros::shutdown();
//			res.res = "stopping recorder";
//
//		} else {
//
//			b_record = true;
//			res.res = "starting recorder";
//
//		}
//
//		return true;
//
//	} else if (cmd == "stop") {
//
//		res.res = "stopping recorder";
//		b_record = false;
//		stopped = true;
//		return true;
//
//	} else {
//
//		res.res = "No such command[" + cmd + "] in [Record::string_command]";
//		ROS_WARN_STREAM(res.res);
//		return false;
//
//	}
//}
