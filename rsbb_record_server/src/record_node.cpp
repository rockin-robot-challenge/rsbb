/**
 *
 * TEST:
 *        files and folders with same name
 *        params
 *   DONE links in paths
 *   DONE overwriting of bagfiles
 *
 *
 * TODO:
 *   refactor package name to rsbb_rosbag_record_server
 *   check bag status and warn if bag not growing enough
 *
 */

#include <ros/ros.h>
#include "record.h"

#include <rsbb_benchmarking_messages/RecordRequest.h>
#include <rsbb_benchmarking_messages/StopRecordRequest.h>
#include <rsbb_benchmarking_messages/SystemStatus.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
using namespace std;
using namespace rsbb_benchmarking_messages;

#define RSBB_SYSTEM_STATUS_MESSAGES_TOPIC_NAME "rsbb_system_status/record_server"
#define RECORD_REQUEST_TOPIC_NAME "record_server/record_request"
#define STOP_RECORD_REQUEST_SERVICE_NAME "record_server/stop_record_request"
#define MESSAGE_TIMEOUT_SECONDS 1.0

Record* recorder_ = NULL;

ros::Publisher system_status_publisher_; // note that current_system_status_.status is set to NORMAL at the start of the node, and to ERROR whenever an error occurs, this because the status can only go from NORMAL to ERROR, and when an error occurs the node is restarted after publishing the system status
SystemStatus current_system_status_;
RecordRequest current_record_request_;
fs::path current_record_dir_path_;
ros::Time last_request_message_;
bool recording_ = false;
bool requested_to_record_ = false;
bool requested_to_stop_ = false;

bool not_equals(RecordRequest a, RecordRequest b) {
	return a.record != b.record || a.benchmark_code != b.benchmark_code || a.robot != b.robot || a.run != b.run || a.team != b.team;
}

string trim(const string& str, const string& whitespace = " \t") {
	const auto strBegin = str.find_first_not_of(whitespace);
	if (strBegin == string::npos)
		return "";

	const auto strEnd = str.find_last_not_of(whitespace);
	const auto strRange = strEnd - strBegin + 1;

	return str.substr(strBegin, strRange);
}

string normalize(string path) {
	string trimmed_path = trim(path, " ");

	if (trimmed_path[0] == '~') {
		char* home = getenv("HOME");
		return home + trimmed_path.substr(1);
	} else {
		return trimmed_path;
	}

}

bool is_valid_dir_path(fs::path p) {
	return fs::exists(p) && fs::is_directory(p);
}

bool check_and_make_dir(fs::path p) {

	if (!fs::exists(p)) {

		// if a file or directory with path p does not exists, make directory p
		if (!fs::create_directory(p)) {
			ROS_ERROR_STREAM("Directory [" << p << "] could not be created. Please, manually create the directory.");
			return false;
		}

		return true;

	} else {
		// if a file or directory with path p does not exist...

		if (fs::is_directory(p)) {

			// and it is a directory, then nothing to be done
			return true;

		} else if (fs::is_regular_file(p)) {

			// and it is a file, rename the file
			fs::rename(p, fs::path(p.string() + "_renamed_file"));
			ROS_WARN_STREAM("Renaming file [" << p << "] to create directory");

			// then create the directory p
			if (!fs::create_directory(p)) {
				ROS_ERROR_STREAM("Directory [" << p << "] could not be created. Please, manually create the directory.");
				return false;
			}

			return true;

		} else {

			ROS_ERROR_STREAM("Directory [" << p << "] could not be created. Please, manually create the directory.");
			return false;

		}

	}

}

void publish_system_status(string d = "") {
	current_system_status_.header.stamp = ros::Time::now();
	current_system_status_.status_description = d;
	system_status_publisher_.publish(current_system_status_);
}

void restart_node() {

	publish_system_status("restarting");

	if (recorder_) {

		recorder_->stop_recording();

	} else {

		if (recording_) {

			ROS_ERROR_STREAM("trying to stop recording but recorder not initialised");

			current_system_status_.status = SystemStatus::ERROR;
			publish_system_status("restarting");

		}

		ros::shutdown();

	}

	//TODO: throw exception just to be sure the node is killed in both cases
}

void timer_callback(const ros::TimerEvent&) {
	// check, if recording, how much time has passed since the last request. in case of a long time print a warning
	if (recording_ && (ros::Time::now() - last_request_message_ > ros::Duration(MESSAGE_TIMEOUT_SECONDS))) {
		ROS_WARN("MESSAGE_TIMEOUT!");
	}

	// check if there was a stop request, in such case stop recording
	if (recording_ && requested_to_stop_) {
		ROS_INFO_STREAM("stopping recording");
		restart_node();
	}

	// check, if recording, that the directory where the bags are saved exists
	if (recording_ && !is_valid_dir_path(current_record_dir_path_)) {
		ROS_ERROR_STREAM("DATA LOST: current record directory not valid! current record directory [" << current_record_dir_path_ << "]");
		current_system_status_.status = SystemStatus::ERROR;
		restart_node();
	}

	//TODO: check bag status and warn if bag not growing normally

	publish_system_status(recording_ ? "recording" : "waiting");
}

void record_request_callback(RecordRequest::ConstPtr record_request_message) {

//	if (!recording_) {
	if (!requested_to_record_) {
		// if NOT recording before

		current_record_request_ = *record_request_message;

		if (current_record_request_.record) {
			ROS_INFO_STREAM("requested to start recording:" << endl << current_record_request_);
			last_request_message_ = ros::Time::now();
//			recording_ = true;
			requested_to_record_ = true;
		}

	} else {
		// if recording before

		last_request_message_ = ros::Time::now();

		if (not_equals(current_record_request_, *record_request_message)) {
			// if the record request changed

			current_record_request_ = *record_request_message;
			ROS_INFO_STREAM("new record request:" << endl << current_record_request_);

			if (current_record_request_.record) {
				// if the request is still to record but a different bag...

				ROS_INFO_STREAM("record request changed. restarting node");
				requested_to_stop_ = true;
				restart_node();

			} else {
				// requested to stop recording
				ROS_INFO_STREAM("requested to stop recording");

				requested_to_stop_ = true;
				restart_node();

			}

		} else {
			// if current_record_request_ didn't change, no need to do anything
			return;
		}

	}

}

bool stop_record_callback(StopRecordRequest::Request& req, StopRecordRequest::Response& res) {

	ROS_INFO_STREAM("requested to stop recording");
	res.result = requested_to_record_;
	if (requested_to_record_)
		requested_to_stop_ = true;

	return true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "record_node");
	ros::NodeHandle nh, pnh("~");
	ROS_INFO("Starting record node");

	system_status_publisher_ = nh.advertise<SystemStatus>(RSBB_SYSTEM_STATUS_MESSAGES_TOPIC_NAME, 100);
	ros::Subscriber record_request_subscriber = nh.subscribe(RECORD_REQUEST_TOPIC_NAME, 100, record_request_callback);
	ros::ServiceServer stop_record_server = nh.advertiseService(STOP_RECORD_REQUEST_SERVICE_NAME, stop_record_callback);
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);
	ros::Rate r(100); // 100 Hz
	string base_path_str;
	vector<string> topics_list;

	// publish current status
	current_system_status_.status = SystemStatus::NORMAL;
	publish_system_status("starting up");

	// preset the record request variables to sensible values. In a normal case these should be overridden by an actual request.
	current_record_request_.record = false;
	current_record_request_.benchmark_code = "undefined";
	current_record_request_.team = "undefined";
	current_record_request_.robot = "";
	current_record_request_.run = 0;

	// get the parameters
	if (!pnh.getParam("general_record_list", topics_list)) {
		ROS_ERROR_STREAM("param topics_list not found or not a list of strings");
		current_system_status_.status = SystemStatus::ERROR;
		publish_system_status("restarting");
		restart_node();
	}
	if (topics_list.empty()) {
		ROS_ERROR_STREAM("param general_record_list can not be empty");
		current_system_status_.status = SystemStatus::ERROR;
		publish_system_status("restarting");
		restart_node();
	}

	if (!pnh.getParam("base_record_directory", base_path_str)) {
		ROS_ERROR_STREAM("param base_record_directory not found");
		current_system_status_.status = SystemStatus::ERROR;
		publish_system_status("restarting");
		restart_node();
	}

	// wait until a RecordRequest message is received
	ROS_INFO_STREAM("Waiting until a record request is received");
	while (!requested_to_stop_ && !requested_to_record_ && ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}

	if(requested_to_stop_){
		publish_system_status("restarting");
		restart_node();
	}

	if (!requested_to_record_ || !ros::ok())
		return 0;

	ROS_INFO_STREAM("Finished waiting");

	// check and, if needed, create the directories for the bag

	// base path
	fs::path base_path(normalize(base_path_str));

	// check base path
	if (!is_valid_dir_path(base_path)) {
		ROS_ERROR_STREAM("Base directory [" << base_path << "] does not exist. Please update the base_directory parameter with the directory where the bags should be saved");
		current_system_status_.status = SystemStatus::ERROR;
		publish_system_status("restarting");
		restart_node();
	}

	// logs directory and path
	//   note that this directory is necessary to ensure that the logs and the score files are saved in different directories,
	//   avoiding the race condition on the check and creation of directories
	fs::path logs_dir("rsbb_logs");
	fs::path logs_path = base_path / logs_dir;

	// check benchmark path
	if (!check_and_make_dir(logs_path)) {
		current_system_status_.status = SystemStatus::ERROR;
		publish_system_status("restarting");
		restart_node();
	}

	// benchmark directory and path
	fs::path bm_dir(trim(current_record_request_.benchmark_code));
	fs::path bm_path = logs_path / bm_dir;

	// check benchmark path
	if (!check_and_make_dir(bm_path)) {
		current_system_status_.status = SystemStatus::ERROR;
		publish_system_status("restarting");
		restart_node();
	}

	// team directory and path
	fs::path team_dir(trim(current_record_request_.team));
	fs::path team_path = bm_path / team_dir;

	// check team path
	if (!check_and_make_dir(team_path)) {
		current_system_status_.status = SystemStatus::ERROR;
		publish_system_status("restarting");
		restart_node();
	}

	// bag (robot) directory and path
	fs::path robot_dir(trim(current_record_request_.robot));
	fs::path bag_path = team_path / robot_dir;
	current_record_dir_path_ = bag_path;

	// check bag path
	if (!check_and_make_dir(bag_path)) {
		current_system_status_.status = SystemStatus::ERROR;
		publish_system_status("restarting");
		restart_node();
	}

	// bag filename prefix and path with the bag's filename prefix
	fs::path bag_filename_prefix(trim(string("run_") + to_string(current_record_request_.run)));
	fs::path bag_filename_prefixed_path = bag_path / bag_filename_prefix;

	// fill the topics list with the topics in the record request
	for(string t: current_record_request_.topics){
		topics_list.push_back(t);
	}

	// set the options for the recorder
	rosbag::RecorderOptions options;
	options.prefix = bag_filename_prefixed_path.string();
	options.topics = topics_list;

	ROS_INFO_STREAM("base_path:           " << base_path);
	ROS_INFO_STREAM("logs_path:           " << logs_path);
	ROS_INFO_STREAM("bm_path:             " << bm_path);
	ROS_INFO_STREAM("team_path:           " << team_path);
	ROS_INFO_STREAM("bag_path:            " << bag_path);
	ROS_INFO_STREAM("bag_filename_prefix: " << bag_filename_prefix);
	ROS_INFO_STREAM("bag_prefixed_path:   " << bag_filename_prefixed_path);
	ROS_INFO_STREAM("options.prefix:      " << options.prefix);
	ROS_INFO_STREAM("options.topics:");
	for (auto t : options.topics)
		ROS_INFO_STREAM("\t" << t);

	recording_ = true;
	ROS_INFO_STREAM("Starting to record");

	// run the recorder node (rosbag record wrapper)
	recorder_ = new Record(nh, options);
	int ret = recorder_->run();

	return ret;
}
