/*
 * Copyright 2014 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of RoAH RSBB.
 *
 * RoAH RSBB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoAH RSBB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoAH RSBB.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __CORE_ZONE_EXEC_H__
#define __CORE_ZONE_EXEC_H__

#include "core_includes.h"

#include "core_shared_state.h"
#include "core_zone_base.h"

#define SYSTEM_STATUS_TIMEOUT_SECONDS 1.0

using namespace rsbb_benchmarking_messages;
//typedef actionlib::SimpleActionClient<ExecuteBenchmarkAction> BenchmarkClient;

class ExecutingBenchmark: boost::noncopyable {
protected:
	CoreSharedState& ss_;

	Publisher timeout_pub_;

	Event const& event_;

	DisplayText display_log_;
	DisplayText display_online_data_;

	roah_rsbb_msgs::BenchmarkState::State state_;
	enum {
		PHASE_PRE, PHASE_EXEC, PHASE_POST
	} phase_;
	bool stopped_due_to_timeout_;
	TimeControl timeout_timer_;
	Time last_stop_time_;
	string state_desc_;
	Time state_time_;

	string manual_operation_;

	RsbbLog log_;

	vector<ScoringItem> scoring_;

	void set_state(Time const& now, roah_rsbb_msgs::BenchmarkState::State const& state, string const& desc) {
		state_ = state;
		state_desc_ = desc;
		state_time_ = now;

		log_.set_state(now, state, desc);
	}

	virtual void phase_exec_2(Time const& now) {
	}

	virtual void phase_exec(string const& desc) {
		Time now = Time::now();

		cout << "phase_exec" << endl;

		if (phase_ == PHASE_PRE) {
			timeout_timer_.start_reset(now);
			cout << "phase_exec:\t\t time_.start_reset" << endl << endl;
		} else {
			timeout_timer_.resume_hot(now);
			cout << "phase_exec:\t\t time_.resume_hot" << endl << endl;
		}
		phase_ = PHASE_EXEC;
		stopped_due_to_timeout_ = false;
		set_state(now, roah_rsbb_msgs::BenchmarkState_State_PREPARE, desc);

		phase_exec_2(now);
	}

	virtual void phase_post_2(Time const& now) {
	}

	virtual void phase_post(string const& desc) {
		Time now = Time::now();

		cout << "AT PHASE POST --- REASON: " << desc << endl;

		phase_ = PHASE_POST;
		last_stop_time_ = now;
		set_state(now, roah_rsbb_msgs::BenchmarkState_State_STOP, desc);

		timeout_timer_.stop_pause(now);
		cout << "phase_post:\t\t time_.stop_pause" << endl << endl;

		phase_post_2(now);
	}

	boost::function<void()> end_;
private:

	virtual void timeout_2() {
		if (phase_ != PHASE_EXEC) {
			return;
		}

		stopped_due_to_timeout_ = true;
		phase_post("Stopped due to timeout!");

		timeout_pub_.publish(std_msgs::Empty());
	}

public:
	ExecutingBenchmark(CoreSharedState& ss, Event const& event, boost::function<void()> end) :
		ss_(ss), timeout_pub_(ss_.nh.advertise<std_msgs::Empty> ("/timeout", 1, false)), event_(event), display_log_(), display_online_data_(), phase_(PHASE_PRE),
				stopped_due_to_timeout_(false), timeout_timer_(ss, event_.benchmark.timeout, boost::bind(&ExecutingBenchmark::timeout_2, this)), manual_operation_(""),
				log_(event.team, event.round, event.run, ss.run_uuid, display_log_), scoring_(event.benchmark.scoring), end_(end) {
		Time now = Time::now();

		set_state(now, roah_rsbb_msgs::BenchmarkState_State_STOP, "All OK for start");
	}

	virtual ~ExecutingBenchmark() {
		log_.end();
	}

	virtual void terminate_benchmark() {
		timeout_timer_.stop_pause(Time());
		cout << "terminate_benchmark:\t\ttime_.stop_pause" << ": " << to_qstring(timeout_timer_.get_until_timeout(Time::now())).toStdString() << endl << endl;

		stop_communication();
		end_();
		cout << "benchmark terminated" << endl;

	}

	virtual void set_score(roah_rsbb::Score const& score) {
		Time now = Time::now();

		for (ScoringItem& i : scoring_) {
			if ( (score.group == i.group) && (score.desc == i.desc)) {
				i.current_value = score.value;
				log_.log_score ("/rsbb_log/score", now, score);
				return;
			}
		}
		ROS_ERROR_STREAM("Did not find group " << score.group << " desc " << score.desc);
	}

	virtual bool manual_operation_complete() {
		ROS_WARN_STREAM("Ignored unexpected manual operation command");
		return false;
	}
	virtual bool manual_operation_complete(string result) {
		ROS_WARN_STREAM("Ignored unexpected manual operation command");
		return false;
	}

	virtual void omf_complete() {
		ROS_WARN_STREAM("Ignored unexpected omf_complete command");
	}

	virtual void omf_damaged(uint8_t damaged) {
		ROS_WARN_STREAM("Ignored unexpected omf_damaged command");
	}

	virtual void omf_button(uint8_t button) {
		ROS_WARN_STREAM("Ignored unexpected omf_button command");
	}

	virtual void start() {
		switch (state_) {
		case roah_rsbb_msgs::BenchmarkState_State_STOP:
			phase_exec("Robot preparing for task");
			return;
		case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
		case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
		case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
			return;
		}
	}

	virtual void stop() {
		switch (state_) {
		case roah_rsbb_msgs::BenchmarkState_State_STOP:
			terminate_benchmark();
			return;
		case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
		case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
		case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
			phase_post("Benchmark Stopped by referee");
			return;
		}
	}

	virtual void fill_2(Time const& now, roah_rsbb::ZoneState& zone) = 0;

	virtual void fill(Time const& now, roah_rsbb::ZoneState& zone) {
		switch (phase_) {
		case PHASE_PRE:
			zone.timer = event_.benchmark.timeout;
			break;
		case PHASE_EXEC:
			zone.timer = timeout_timer_.get_until_timeout(now);
			break;
		case PHASE_POST:
			zone.timer = last_stop_time_ + Duration(param_direct<double> ("~after_stop_duration", 120.0)) - now;
			break;
		}

		zone.state = state_desc_;

		zone.manual_operation = manual_operation_;

		zone.start_enabled = state_ == roah_rsbb_msgs::BenchmarkState_State_STOP;
		zone.stop_enabled = !zone.start_enabled && phase_ == PHASE_EXEC;

		const size_t log_size = param_direct<int> ("~display_log_size", 3000);
		zone.log = display_log_.last(log_size);
		zone.online_data = display_online_data_.last(log_size);

		for (ScoringItem const& i : scoring_) {
			if (zone.scoring.empty() || (zone.scoring.back().group_name != i.group)) {
				zone.scoring.push_back (roah_rsbb::ZoneScoreGroup());
				zone.scoring.back().group_name = i.group;
			}
			switch (i.type) {
				case ScoringItem::SCORING_BOOL:
				zone.scoring.back().types.push_back (roah_rsbb::ZoneScoreGroup::SCORING_BOOL);
				break;
				case ScoringItem::SCORING_UINT:
				zone.scoring.back().types.push_back (roah_rsbb::ZoneScoreGroup::SCORING_UINT);
				break;
				default:
				ROS_FATAL_STREAM ("type in ScoringItem error");
				abort_rsbb();
			}
			zone.scoring.back().descriptions.push_back (i.desc);
			zone.scoring.back().current_values.push_back (i.current_value);
		}

		fill_2(now, zone);
	}

	roah_rsbb_msgs::BenchmarkState::State state() {
		return state_;
	}

	virtual void
	stop_communication() = 0;
};

class ExecutingSingleRobotBenchmark: public ExecutingBenchmark {
protected:
	string robot_name_;

	unique_ptr<roah_rsbb::RosPrivateChannel> private_channel_;

	roah_rsbb_msgs::Time ack_;
	Duration last_skew_;
	Time last_beacon_;

	Timer state_timer_;

	uint32_t messages_saved_;

	ReceiverRepeated rcv_notifications_;
	ReceiverRepeated rcv_activation_event_;
	ReceiverRepeated rcv_visitor_;
	ReceiverRepeated rcv_final_command_;

	virtual void receive_robot_state_2(Time const& now, roah_rsbb_msgs::RobotState const& msg) {
	}

	virtual void fill_benchmark_state_2(roah_rsbb_msgs::BenchmarkState& msg) {
	}

private:
	void transmit_state(const TimerEvent& = TimerEvent()) {
		ROS_DEBUG("Transmitting benchmark state");

		roah_rsbb_msgs::BenchmarkState msg;
		msg.set_benchmark_type(event_.benchmark_code);
		msg.set_benchmark_state(state_);
		(*(msg.mutable_acknowledgement())) = ack_;
		fill_benchmark_state_2(msg);
		private_channel_->send(msg);
	}

	void receive_benchmark_state(boost::asio::ip::udp::endpoint endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<const roah_rsbb_msgs::BenchmarkState> msg) {
		ROS_ERROR_STREAM(
				"Detected another RSBB transmitting in the private channel for team " << event_.team << ": " << endpoint.address().to_string() << ":" << endpoint.port()
						<< ", COMP_ID " << comp_id << ", MSG_TYPE " << msg_type << endl);
	}

	void receive_robot_state(boost::asio::ip::udp::endpoint endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<const roah_rsbb_msgs::RobotState> msg) {
		Time now = last_beacon_ = Time::now();
		Time msg_time(msg->time().sec(), msg->time().nsec());
		Duration last_skew_ = msg_time - now;

		ROS_DEBUG_STREAM(
				"Received RobotState from " << endpoint.address().to_string() << ":" << endpoint.port() << ", COMP_ID " << comp_id << ", MSG_TYPE " << msg_type << ", time: "
						<< msg->time().sec() << "." << msg->time().nsec() << ", skew: " << last_skew_);

		ss_.active_robots.add(event_.team, robot_name_, last_skew_, now);

		messages_saved_ = msg->messages_saved();
		/* if ( (messages_saved_ == 0) */
		/*      && (param_direct<bool> ("~check_messages_saved", true)) */
		/*      && ( (state_ == roah_rsbb_msgs::BenchmarkState_State_GOAL_TX) */
		/*           || (state_ == roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT)) */
		/*      && ( (now - state_time_).toSec() > param_direct<double> ("~check_messages_saved_timeout", 5.0))) { */
		/*   phase_post ("STOPPED BENCHMARK: Messages saved information received from robot is still 0!"); */
		/* } */

		ack_ = msg->time();

		rcv_notifications_.receive(now, msg->notifications());
		rcv_activation_event_.receive(now, msg->activation_event());
		rcv_visitor_.receive(now, msg->visitor());
		rcv_final_command_.receive(now, msg->final_command());

		receive_robot_state_2(now, *msg);
	}

public:
	ExecutingSingleRobotBenchmark(CoreSharedState& ss, Event const& event, boost::function<void()> end, string const& robot_name) :
				ExecutingBenchmark(ss, event, end),
				robot_name_(robot_name),
				private_channel_(new roah_rsbb::RosPrivateChannel(
						param_direct<string> ("~rsbb_broadcast_address", "10.0.0.255"),
						ss_.private_port(),
						event_.password,
						param_direct<string> ("~rsbb_cypher", "aes-128-cbc")
						)),
				state_timer_(ss_.nh.createTimer(Duration(0.2), &ExecutingSingleRobotBenchmark::transmit_state, this)),
				messages_saved_(0),
				rcv_notifications_(log_, "/notification", display_online_data_),
				rcv_activation_event_(log_, "/command", display_online_data_),
				rcv_visitor_(log_, "/visitor", display_online_data_),
				rcv_final_command_(log_, "/command", display_online_data_)
	{
		ack_.set_sec(0);
		ack_.set_nsec(0);
		private_channel_->set_benchmark_state_callback(&ExecutingSingleRobotBenchmark::receive_benchmark_state, this);
		private_channel_->set_robot_state_callback(&ExecutingSingleRobotBenchmark::receive_robot_state, this);
		ss_.benchmarking_robots[event_.team] = make_pair(robot_name_, private_channel_->port());
	}

	void stop_communication() {
		state_timer_.stop();
		private_channel_->signal_benchmark_state_received().disconnect_all_slots();
		private_channel_->signal_robot_state_received().disconnect_all_slots();
		ss_.benchmarking_robots.erase(event_.team);
	}
};

class ExecutingSimpleBenchmark: public ExecutingSingleRobotBenchmark {

	Timer refbox_state_publish_timer_;

	ServiceClient stop_record_service_client_;
 	Publisher record_request_publisher_;
	RecordRequest record_request_;

	void receive_robot_state_2(Time const& now, roah_rsbb_msgs::RobotState const& msg) {
		switch (state_) {
		case roah_rsbb_msgs::BenchmarkState_State_STOP:
			break;
		case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
			if (msg.robot_state() == roah_rsbb_msgs::RobotState_State_WAITING_GOAL) {
				set_state(now, roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT, "Robot finished preparation, executing (no explicit goal)");
			}
			break;
		case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
			ROS_FATAL_STREAM("Internal error, state should never be BenchmarkState_State_GOAL_TX for this benchmark");
			terminate_benchmark();
			return;
		case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
			switch (msg.robot_state()) {
			case roah_rsbb_msgs::RobotState_State_STOP:
			case roah_rsbb_msgs::RobotState_State_PREPARING:
				phase_exec("Received wrong state from robot, retrying from prepare");
				break;
			case roah_rsbb_msgs::RobotState_State_WAITING_GOAL:
			case roah_rsbb_msgs::RobotState_State_EXECUTING:
				// Keep
				break;
			case roah_rsbb_msgs::RobotState_State_RESULT_TX:
				phase_post("Benchmark completed by the robot");
				break;
			}
			break;
		}

		if (event_.benchmark.commands_devices) {
			if (msg.has_devices_switch_1() && (msg.devices_switch_1() != ss_.last_devices_state->switch_1)) {
				roah_devices::Bool b;
				b.request.data = msg.devices_switch_1();
				call_service("/devices/switch_1/set", b);
				log_.log_uint8("/rsbb_log/devices/switch_1", now, b.request.data ? 1 : 0);
			}
			if (msg.has_devices_switch_2() && (msg.devices_switch_2() != ss_.last_devices_state->switch_2)) {
				roah_devices::Bool b;
				b.request.data = msg.devices_switch_2();
				call_service("/devices/switch_2/set", b);
				log_.log_uint8("/rsbb_log/devices/switch_2", now, b.request.data ? 1 : 0);
			}
			if (msg.has_devices_switch_3() && (msg.devices_switch_3() != ss_.last_devices_state->switch_3)) {
				roah_devices::Bool b;
				b.request.data = msg.devices_switch_3();
				call_service("/devices/switch_3/set", b);
				log_.log_uint8("/rsbb_log/devices/switch_3", now, b.request.data ? 1 : 0);
			}
			if (msg.has_devices_blinds() && (msg.devices_blinds() != ss_.last_devices_state->blinds)) {
				roah_devices::Percentage p;
				p.request.data = msg.devices_blinds();
				call_service("/devices/blinds/set", p);
				log_.log_uint8("/rsbb_log/devices/blinds", now, p.request.data);
			}
			if (msg.has_devices_dimmer() && (msg.devices_dimmer() != ss_.last_devices_state->dimmer)) {
				roah_devices::Percentage p;
				p.request.data = msg.devices_dimmer();
				call_service("/devices/dimmer/set", p);
				log_.log_uint8("/rsbb_log/devices/dimmer", now, p.request.data);
			}
			if (msg.has_tablet_display_map() && (ss_.tablet_display_map != msg.tablet_display_map())) {
				ss_.tablet_display_map = msg.tablet_display_map();
				log_.log_uint8("/rsbb_log/tablet/display_map", now, ss_.tablet_display_map ? 1 : 0);
			}
		}
	}

	void terminate_benchmark() {
		timeout_timer_.stop_pause(Time());
		cout << "terminate_benchmark:\t\ttime_.stop_pause" << ": " << to_qstring(timeout_timer_.get_until_timeout(Time::now())).toStdString() << endl << endl;

		StopRecordRequest stop_record_request;
		if (stop_record_service_client_.call(stop_record_request)){
			ROS_INFO("Called stop_record service");
			if(!stop_record_request.response.result) ROS_INFO("record server refused to stop recording (probably because it was already not recording)");
		}else{
			ROS_ERROR("Failed to call stop_record service");
		}

		record_request_ = RecordRequest();
		record_request_.record = false;
		record_request_publisher_.publish(record_request_);

		stop_communication();
		end_();
		cout << "benchmark terminated" << endl;

	}

	void refbox_state_publish_timer_callback(const TimerEvent& = TimerEvent()) {
		if(record_request_publisher_) record_request_publisher_.publish(record_request_);
	}

public:
	ExecutingSimpleBenchmark(CoreSharedState& ss, Event const& event, boost::function<void()> end, string const& robot_name, bool do_record = true) :
		ExecutingSingleRobotBenchmark(ss, event, end, robot_name),
		refbox_state_publish_timer_(ss_.nh.createTimer(Duration(0.1), &ExecutingSimpleBenchmark::refbox_state_publish_timer_callback, this)) {

		if(do_record){
			try{
				stop_record_service_client_ = ss_.nh.serviceClient<StopRecordRequestRequest>("record_server/stop_record_request");
				record_request_publisher_ = ss_.nh.advertise<RecordRequest> ("record_server/record_request", 1, true);


				// publish the record request for the rosbag record server
				record_request_.record = true;
				record_request_.benchmark_code = event_.benchmark_code;
				record_request_.team = event_.team;
//				record_request_.robot = "";
				record_request_.run = event_.run;
				record_request_.topics = event_.benchmark.record_topics;
				record_request_publisher_.publish(record_request_);

			} catch (const std::exception& exc) {
				ROS_ERROR_STREAM("Could not advertise record services and topics");
			}
		}

	}

	void fill_2(Time const& now, roah_rsbb::ZoneState& zone) {
		add_to_sting(zone.state) << "Messages saved: " << messages_saved_;

		if (last_skew_ > Duration(0.5)) {
			zone.state += "\nWARNING: Last clock skew above threshold: " + to_string(last_skew_.toSec());
		}
		if ((now - last_beacon_) > Duration(5)) {
			zone.state += "\nWARNING: Last robot transmission received " + to_string((now - last_beacon_).toSec()) + " seconds ago";
		}
	}

};









/*
 * Functions:
 *
 *
 * → Internal control functions:
 *
 *   - goal control functions:
 * void begin_goal_execution();
 * void end_goal_execution();
 *
 *   - phase control functions:
 * void phase_exec(string const& desc);
 * void phase_post(string const& desc);
 *
 *   - timeout control functions:
 * void goal_timeout_callback();
 * void global_timeout_callback();
 *
 *
 * → GUI control functions:
 * void start();
 * void stop();
 * void manual_operation_complete(string result);
 *
 *
 * → Benchmark Script communication functions:
 * bool execute_manual_operation_callback(ExecuteManualOperation::Request& req, ExecuteManualOperation::Response& res);
 * bool execute_goal_callback(ExecuteGoal::Request& req, ExecuteGoal::Response& res);
 * bool end_goal_callback(EndGoal::Request& req, EndGoal::Response& res);
 * bool end_benchmark_callback(EndBenchmark::Request& req, EndBenchmark::Response& res);
 *
 *
 * → Robot Communication functions:
 * void receive_robot_state_2(Time const& now, roah_rsbb_msgs::RobotState const& msg);
 *
 *
 */

class ExecutingExternallyControlledBenchmark: public ExecutingSingleRobotBenchmark {

	RefBoxState refbox_state_;

	string benchmark_payload_;
	string goal_execution_payload_;
	string manual_operation_payload_;

	Timer refbox_state_publish_timer_, check_status_timer_;
	Publisher refbox_state_pub_;
	Publisher system_status_publisher_;
	Subscriber bmbox_state_subscriber_, bmbox_status_subscriber_, record_server_status_subscriber_;

	// Incoming script server services
	ServiceServer execute_manual_operation_service_, execute_goal_service_, end_benchmark_service_;

	// Outgoing script server services
	ServiceClient init_benchmark_service_, terminate_benchmark_service_;

	// Outgoing script services
	ServiceClient start_benchmark_service_, manual_operation_complete_service_, goal_execution_started_service_, goal_complete_service_, referee_score_service_, stop_benchmark_service_;

	// Outgoing record server services and topics
	ServiceClient stop_record_service_client_;
 	Publisher record_request_publisher_;
	RecordRequest record_request_;

	SystemStatus current_system_status_, last_bmbox_status_, last_record_server_status_;

	string current_goal_payload_;
	float current_goal_timeout_ = 0.0;

	TimeControl timeout_timer_;
	TimeControl global_timeout_timer_;

	BmBoxState::ConstPtr last_bmbox_state_;
	BmBoxState::_state_type printStates_last_last_bmbox_state_;
	RefBoxState::_benchmark_state_type printStates_last_benchmark_phase_;
	RefBoxState::_goal_execution_state_type printStates_last_goal_execution_state_;
	RefBoxState::_manual_operation_state_type printStates_last_manual_operation_state_;
	roah_rsbb_msgs::BenchmarkState::State printStates_last_benchmark_state_;
	roah_rsbb_msgs::RobotState::State printStates_last_robot_state_ = roah_rsbb_msgs::RobotState_State::RobotState_State_PREPARING, printStates_robot_state_;


public:
	ExecutingExternallyControlledBenchmark(CoreSharedState& ss, Event const& event, boost::function<void()> end, string const& robot_name) :
		ExecutingSingleRobotBenchmark(ss, event, end, robot_name),

		refbox_state_publish_timer_(ss_.nh.createTimer(Duration(0.1), &ExecutingExternallyControlledBenchmark::refbox_state_publish_timer_callback, this)),
		check_status_timer_(ss_.nh.createTimer(Duration(0.1), &ExecutingExternallyControlledBenchmark::check_status_timer_callback, this)),

		refbox_state_pub_(ss_.nh.advertise<RefBoxState> ("bmbox/refbox_state", 1, true)),
		system_status_publisher_(ss_.nh.advertise<SystemStatus>("rsbb_system_status/refbox/benchmark", 100)),
		bmbox_state_subscriber_(ss_.nh.subscribe("bmbox/bmbox_state", 1, &ExecutingExternallyControlledBenchmark::bmbox_state_callback, this)),
		bmbox_status_subscriber_(ss_.nh.subscribe("rsbb_system_status/bmbox", 1, &ExecutingExternallyControlledBenchmark::bmbox_status_callback, this)),
		record_server_status_subscriber_(ss_.nh.subscribe("rsbb_system_status/record_server", 1, &ExecutingExternallyControlledBenchmark::record_server_status_callback, this)),

		execute_manual_operation_service_(ss_.nh.advertiseService("bmbox/execute_manual_operation", &ExecutingExternallyControlledBenchmark::execute_manual_operation_callback, this)),
		execute_goal_service_(ss_.nh.advertiseService("bmbox/execute_goal", &ExecutingExternallyControlledBenchmark::execute_goal_callback, this)),
		end_benchmark_service_(ss_.nh.advertiseService("bmbox/end_benchmark", &ExecutingExternallyControlledBenchmark::end_benchmark_callback, this)),

		init_benchmark_service_(ss_.nh.serviceClient<InitBenchmark>("bmbox/init_benchmark")),
		terminate_benchmark_service_(ss_.nh.serviceClient<TerminateBenchmark>("bmbox/terminate_benchmark")),

		start_benchmark_service_(ss_.nh.serviceClient<StartBenchmark>("bmbox/start_benchmark")),
		manual_operation_complete_service_(ss_.nh.serviceClient<ManualOperationComplete>("bmbox/manual_operation_complete")),
		goal_execution_started_service_(ss_.nh.serviceClient<GoalStarted>("bmbox/goal_execution_started")),
		goal_complete_service_(ss_.nh.serviceClient<GoalComplete>("bmbox/goal_complete")),
		referee_score_service_(ss_.nh.serviceClient<RefereeScore>("bmbox/referee_score")),
		stop_benchmark_service_(ss_.nh.serviceClient<StopBenchmark>("bmbox/stop_benchmark")),

		stop_record_service_client_(ss_.nh.serviceClient<StopRecordRequestRequest>("record_server/stop_record_request")),
		record_request_publisher_(ss_.nh.advertise<RecordRequest> ("record_server/record_request", 1, true)),

		timeout_timer_(ss, event_.benchmark.timeout, true, boost::bind(&ExecutingExternallyControlledBenchmark::goal_timeout_callback, this)),
		global_timeout_timer_(ss, event_.benchmark.total_timeout, true, boost::bind(&ExecutingExternallyControlledBenchmark::global_timeout_callback, this)),

		last_bmbox_state_(boost::make_shared<BmBoxState>())
	{

		current_system_status_.status = SystemStatus::NORMAL;
		publish_system_status();

		cout << endl << endl << endl << endl << endl << endl << endl << endl << "STARTING BENCHMARK: " << event.benchmark.code << endl << event << endl << endl << endl;


		// call the init_benchmark service of the benchmark script server
		InitBenchmark benchmark_request;
		benchmark_request.request.benchmark_code = event_.benchmark_code;
		benchmark_request.request.team = event_.team;
		benchmark_request.request.robot = "";
		benchmark_request.request.run = event_.run;
		if (init_benchmark_service_.call(benchmark_request) && benchmark_request.response.result){
			ROS_INFO("Called service init_benchmark: %s", (benchmark_request.response.result)?"true":"false");
		}else{
			ROS_ERROR("Failed to call service init_benchmark. terminating benchmark execution.");
			terminate_benchmark();
		}

		// publish the record request for the rosbag record server
		record_request_.record = true;
		record_request_.benchmark_code = event_.benchmark_code;
		record_request_.team = event_.team;
//		record_request_.robot = "";
		record_request_.run = event_.run;
		record_request_.topics = event_.benchmark.record_topics;
		record_request_publisher_.publish(record_request_);

	}



	/***********************************
	 *                                 *
	 *   Internal control functions    *
	 *                                 *
	 ***********************************/

	/*
	 * Starts the current goal
	 */
	void start_goal_execution(){
		printStates();
		cout << "begin_goal_execution" << endl;

		if(phase_ == PHASE_PRE){
			ROS_ERROR("start_goal_execution: Requested goal execution in phase PHASE_PRE");
			return;
		}

		Time now = Time::now();

		// resume the global timeout timer, that needs to tick when the robot is executing a goal (the prepare time counts for the timeout)
		global_timeout_timer_.resume(now);
		cout << "start_goal_execution:\t\t global_timeout_timer_.resume" << ": " << to_qstring(global_timeout_timer_.get_until_timeout(Time::now())).toStdString() << endl << endl;

		// reset the goal timeout with the new value for this goal
		if(current_goal_timeout_ > 0){

			// If a goal timeout was provided by the benchmark script, use it
			timeout_timer_.start_reset(now, Duration(current_goal_timeout_));
			cout << "start_goal_execution: setting bmbox timeout:\t" << to_qstring(timeout_timer_.get_until_timeout(Time::now())).toStdString() << endl;

		} else if (event_.benchmark.timeout < event_.benchmark.total_timeout) {

			// else, if a goal timeout is specified in the configuration then use this one
			timeout_timer_.start_reset(now, event_.benchmark.timeout);
			cout << "start_goal_execution: setting default goal timeout:\t" << to_qstring(timeout_timer_.get_until_timeout(Time::now())).toStdString() << endl;

		} else {

			// otherwise use the total timeout
			timeout_timer_.start_reset(now, event_.benchmark.total_timeout);
			cout << "start_goal_execution: setting default total timeout:\t" << to_qstring(timeout_timer_.get_until_timeout(Time::now())).toStdString() << endl;
		}

		set_state(now, roah_rsbb_msgs::BenchmarkState_State_PREPARE, "Requested the robot to prepare for a new goal");
		set_goal_execution_state(RefBoxState::TRANSMITTING_GOAL);

	}

	/*
	 * Terminates the current goal (basically stops the timeout timers)
	 */
	void end_goal_execution(bool completed_by_robot = false, bool goal_timeout = false, string result = "") {
		printStates();
		cout << "end_goal_execution" << endl;

		Time now = Time::now();

		timeout_timer_.stop_pause(now);
		global_timeout_timer_.stop_pause(now);
		cout << "end_goal_execution:\t\t time_.stop_pause" << ": " << to_qstring(timeout_timer_.get_until_timeout(now)).toStdString() << endl << endl;
		cout << "end_goal_execution:\t\t global_timeout_timer_.stop_pause" << ": " << to_qstring(global_timeout_timer_.get_until_timeout(now)).toStdString() << endl << endl;

		current_goal_payload_ = "";
		current_goal_timeout_ = 0;

	}

	/*
	 * Starts the benchmark by setting the RefBox states
	 */
	void phase_exec(string const& desc) {
		printStates();
		cout << "phase_exec" << endl;

		phase_ = PHASE_EXEC;
		set_benchmark_state(RefBoxState::EXECUTING_BENCHMARK);
		set_goal_execution_state(RefBoxState::READY);
		set_manual_operation_state(RefBoxState::READY);

		send_referee_score();

		StartBenchmark start_benchmark;
		start_benchmark.request.refbox_state = refbox_state_;

		// if the service call fails in any way, abort the benchmark
		if (start_benchmark_service_.call(start_benchmark) && start_benchmark.response.result){
			ROS_INFO("Called bmbox/start_benchmark service");
		}else{
			ROS_ERROR("Failed to call \"bmbox/start_benchmark\" service or bmbox refused to start benchmark. Aborting the benchmark");

			// end the benchmark
			phase_post("Failed to start");
		}

	}

	/*
	 * Terminates the benchmark based on RefBox states
	 * by telling to stop to the robot and to the script, and if the benchmark was completed then destroys everything
	 */
	void phase_post(string const& desc) {
		printStates();
		cout << "phase_post" << endl;

		phase_ = PHASE_POST;
		set_state(Time::now(), roah_rsbb_msgs::BenchmarkState_State_STOP, desc);

	}

	/*
	 * Called when a goal timeout expires
	 */
	void goal_timeout_callback() {
		cout << "timeout_callback" << endl;

		if (phase_ != PHASE_EXEC) return;

		// advertise that a timeout has occurred
		timeout_pub_.publish(std_msgs::Empty());

		set_state(Time::now(), roah_rsbb_msgs::BenchmarkState_State_STOP, "");
		end_goal_execution(false, true, "");

		GoalComplete goal_complete;
		goal_complete.request.goal_result = "";
		goal_complete.request.goal_timeout = true;
		goal_complete.request.refbox_state = get_goal_execution_state_copy(RefBoxState::READY);

		if (goal_complete_service_.call(goal_complete) && goal_complete.response.result) {
			ROS_INFO("Called bmbox/goal_complete service");
			set_goal_execution_state(RefBoxState::READY);
		}else{

			Rate r(5);
			for(int i=0; i<10; i++){
				ROS_ERROR("Failed to call \"bmbox/goal_complete\" service or bmbox refused to acknowledge that goal has timed out");

				if (goal_complete_service_.call(goal_complete) && goal_complete.response.result){
					set_goal_execution_state(RefBoxState::READY);
//					set_goal_execution_state(RefBoxState::ERROR); // use ERROR state instead?
					break;
				}
				r.sleep();
			}

		}

		set_goal_execution_state(RefBoxState::READY);

	}

	/*
	 * Called when the global timeout expires
	 */
	void global_timeout_callback() {
		cout << "global_timeout_callback" << endl;

		if (phase_ != PHASE_EXEC) return;

		// terminate the benchmark by calling phase_post
		clear_manual_operation();
		end_goal_execution();

		set_benchmark_state(RefBoxState::GLOBAL_TIMEOUT);

		phase_post("Stopped due to global timeout!");

		// advertise that a timeout has occurred
		timeout_pub_.publish(std_msgs::Empty());

		StopBenchmark stop_benchmark_request;
		stop_benchmark_request.request.refbox_state = refbox_state_;

		if (stop_benchmark_service_.call(stop_benchmark_request) && stop_benchmark_request.response.result){
			ROS_INFO("Called stop_benchmark service");
		} else {
			ROS_INFO("Tried to call stop_benchmark service but not available or bmbox refused to acknowledge that the benchmark is stopped");
		}

	}

	void fill(Time const& now, roah_rsbb::ZoneState& zone) {

		switch (phase_) {
		case PHASE_PRE:		zone.timer = event_.benchmark.timeout; break;
		case PHASE_EXEC:	zone.timer = timeout_timer_.get_until_timeout(now); break;
		case PHASE_POST:	break;
		}

		zone.manual_operation = manual_operation_;
		zone.state = state_desc_;

		zone.benchmark_state = get_refbox_state_string(refbox_state_.benchmark_state);
		zone.goal_state = get_refbox_state_string(refbox_state_.goal_execution_state);
		zone.manual_operation_state = get_refbox_state_string(refbox_state_.manual_operation_state);
		zone.bmscript_state = get_bmbox_state_string(last_bmbox_state_->state);
		zone.robot_state = get_robot_state_string(printStates_robot_state_);


		if (now - last_bmbox_status_.header.stamp > Duration(SYSTEM_STATUS_TIMEOUT_SECONDS)) {
			zone.state = "WARNING: Not connected to BmBox script. Cannot start.";
			zone.start_enabled = false;
		} else {
			zone.start_enabled = (phase_ == PHASE_PRE);
		}

		zone.stop_enabled  = (phase_ == PHASE_EXEC);

		const size_t log_size = param_direct<int> ("~display_log_size", 3000);
		zone.log = display_log_.last(log_size);
		zone.online_data = display_online_data_.last(log_size);

		if (phase_ == PHASE_EXEC) add_to_sting(zone.state) << "Benchmark timeout: " << to_qstring(global_timeout_timer_.get_until_timeout(Time::now())).toStdString();

		if (refbox_state_pub_.getNumSubscribers() > 1) add_to_sting(zone.state) << "WARNING: multiple RefBox state subscribers";

		for (ScoringItem const& i : scoring_) {
			if (zone.scoring.empty() || (zone.scoring.back().group_name != i.group)) {
				zone.scoring.push_back (roah_rsbb::ZoneScoreGroup());
				zone.scoring.back().group_name = i.group;
			}
			switch (i.type) {
				case ScoringItem::SCORING_BOOL:
				zone.scoring.back().types.push_back (roah_rsbb::ZoneScoreGroup::SCORING_BOOL);
				break;
				case ScoringItem::SCORING_UINT:
				zone.scoring.back().types.push_back (roah_rsbb::ZoneScoreGroup::SCORING_UINT);
				break;
				default:
				ROS_FATAL_STREAM ("type in ScoringItem error");
				abort_rsbb();
			}
			zone.scoring.back().descriptions.push_back (i.desc);
			zone.scoring.back().current_values.push_back (i.current_value);
		}
	}

	void send_referee_score(){

		cout << "send_referee_score" << endl;

		YAML::Node scoring_node;

		for(ScoringItem i: scoring_) scoring_node.push_back(i.to_yaml_node());

		if(scoring_node.size()){
			YAML::Emitter scoring_node_os;

			scoring_node_os << scoring_node;

			RefereeScore referee_score;
			referee_score.request.score = scoring_node_os.c_str();//.as<string>();
			referee_score.request.refbox_state = refbox_state_;

			if (referee_score_service_.call(referee_score) && referee_score.response.success){
				ROS_INFO("Called referee_score service");
			} else {
				ROS_INFO("Tried to call referee_score service but not available or bmbox refused to receive referee score");
			}
		}
	}

	void fill_2(Time const& now, roah_rsbb::ZoneState& zone){}

	void set_benchmark_state(RefBoxState::_benchmark_state_type benchmark_state, string const& benchmark_payload = "") {

		if (refbox_state_.benchmark_state == benchmark_state && benchmark_payload_ == benchmark_payload) return;

		refbox_state_.benchmark_state = benchmark_state;
		refbox_state_.benchmark_payload = benchmark_payload;

		refbox_state_pub_.publish(refbox_state_);

	}

	RefBoxState get_benchmark_state_copy(RefBoxState::_benchmark_state_type benchmark_state, string const& benchmark_payload = "") {

		RefBoxState refbox_state_copy = refbox_state_;

		refbox_state_copy.benchmark_state = benchmark_state;
		refbox_state_copy.benchmark_payload = benchmark_payload;

		return refbox_state_copy;

	}

	void set_goal_execution_state(RefBoxState::_goal_execution_state_type goal_execution_state, string const& goal_execution_payload = "") {

		if (refbox_state_.goal_execution_state == goal_execution_state && refbox_state_.goal_execution_payload == goal_execution_payload) return;

		refbox_state_.goal_execution_state = goal_execution_state;
		refbox_state_.goal_execution_payload = goal_execution_payload;

		refbox_state_pub_.publish(refbox_state_);

	}

	RefBoxState get_goal_execution_state_copy(RefBoxState::_goal_execution_state_type goal_execution_state, string const& goal_execution_payload = "") {

		RefBoxState refbox_state_copy = refbox_state_;

		refbox_state_copy.goal_execution_state = goal_execution_state;
		refbox_state_copy.goal_execution_payload = goal_execution_payload;

		return refbox_state_copy;

	}

	void set_manual_operation_state(RefBoxState::_manual_operation_state_type manual_operation_state, string const& manual_operation_payload = "") {

		if (refbox_state_.manual_operation_state == manual_operation_state && refbox_state_.manual_operation_payload == manual_operation_payload) return;

		refbox_state_.manual_operation_state = manual_operation_state;
		refbox_state_.manual_operation_payload = manual_operation_payload;

		refbox_state_pub_.publish(refbox_state_);

	}

	RefBoxState get_manual_operation_state_copy(RefBoxState::_manual_operation_state_type manual_operation_state, string const& manual_operation_payload = "") {

		RefBoxState refbox_state_copy = refbox_state_;

		refbox_state_copy.manual_operation_state = manual_operation_state;
		refbox_state_copy.manual_operation_payload = manual_operation_payload;

		return refbox_state_copy;

	}

	void publish_system_status(string d = ""){

		current_system_status_.header.stamp = ros::Time::now();
		current_system_status_.status_description = d;
		system_status_publisher_.publish(current_system_status_);

	}

	void refbox_state_publish_timer_callback(const TimerEvent& = TimerEvent()) {

		refbox_state_pub_.publish(refbox_state_);

		record_request_publisher_.publish(record_request_);

		system_status_publisher_.publish(current_system_status_);

	}

	void check_status_timer_callback(const TimerEvent& = TimerEvent()) {
		Time now = Time::now();

		if(last_bmbox_status_.header.stamp > Time(0) && now - last_bmbox_status_.header.stamp > Duration(SYSTEM_STATUS_TIMEOUT_SECONDS )){
			ROS_WARN("bmbox status timeout! (last status received %f seconds ago)", (now - last_bmbox_status_.header.stamp).toSec());
		}

		if(last_record_server_status_.header.stamp > Time(0) && now - last_record_server_status_.header.stamp > Duration(SYSTEM_STATUS_TIMEOUT_SECONDS )){
			ROS_WARN("record server status timeout! (last status received %f seconds ago)", (now - last_record_server_status_.header.stamp).toSec());
		}

	}

	void fill_benchmark_state_2(roah_rsbb_msgs::BenchmarkState& msg) {

		if (state_ != roah_rsbb_msgs::BenchmarkState_State_GOAL_TX) return;

		if(current_goal_payload_.size()) msg.set_generic_goal(current_goal_payload_);

	}

	void clear_manual_operation() {

		manual_operation_.clear();

	}

	void terminate_benchmark() {

		// If the benchmark is not executing (ended, stopped, timed out, etc) terminate, otherwise stop then terminate

		if(refbox_state_.benchmark_state == RefBoxState::EXECUTING_BENCHMARK){
			stop();
		}

		timeout_timer_.stop_pause(Time());

		TerminateBenchmark terminate_benchmark_request;
		if (terminate_benchmark_service_.call(terminate_benchmark_request) && terminate_benchmark_request.response.result){
			ROS_INFO("Called terminate_benchmark service");
		}else{
			ROS_INFO("Tried to call terminate_benchmark service but not available or bmbox refused to terminate benchmark (maybe it was not running a script)");
		}

		StopRecordRequest stop_record_request;
		if (stop_record_service_client_.call(stop_record_request)){
			ROS_INFO("Called stop_record service");
			if(!stop_record_request.response.result) ROS_INFO("record server refused to stop recording (probably because it was already not recording)");
		}else{
			ROS_ERROR("Failed to call stop_record service");
		}

		record_request_ = RecordRequest();
		record_request_.record = false;
		record_request_publisher_.publish(record_request_);

		stop_communication();
		end_();
		cout << "benchmark terminated" << endl;

	}


	/*****************************
	 *                            *
	 *   GUI control functions    *
	 *                            *
	 *****************************/

	void start() {

		phase_exec("");

	}

	void stop() {
		printStates();
		cout << endl << "stop" << endl << endl;

		end_goal_execution();
		clear_manual_operation();

		set_benchmark_state(RefBoxState::STOP);

		phase_post("");

		StopBenchmark stop_benchmark_request;
		stop_benchmark_request.request.refbox_state = refbox_state_;

		if (stop_benchmark_service_.call(stop_benchmark_request) && stop_benchmark_request.response.result){
			ROS_INFO("Called stop_benchmark service");
		} else {
			ROS_INFO("Tried to call stop_benchmark service but not available or bmbox refused to acknowledge that the benchmark is stopped");
		}

	}

	bool manual_operation_complete(string result) {
		printStates();

		if (!(
		    ((state_ == roah_rsbb_msgs::BenchmarkState_State_PREPARE)
		||   (state_ == roah_rsbb_msgs::BenchmarkState_State_STOP)
		||   (state_ == roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT))
		&&   (refbox_state_.manual_operation_state		== RefBoxState::EXECUTING_MANUAL_OPERATION)))
		{
			printStates();
			ROS_ERROR("manual_operation_complete(): could not terminate manual operation: inconsistent states");
			return false;
		}

		ManualOperationComplete manual_operation_complete;
		manual_operation_complete.request.manual_operation_result = result;
		manual_operation_complete.request.refbox_state = get_manual_operation_state_copy(RefBoxState::READY);

		if (manual_operation_complete_service_.call(manual_operation_complete) && manual_operation_complete.response.result){
			ROS_INFO("Called bmbox/manual_operation_complete service");
			set_manual_operation_state(RefBoxState::READY, result);
			clear_manual_operation();
			return true;
		}else{
			ROS_ERROR("Failed to call \"bmbox/manual_operation_complete\" service or bmbox refused to terminate manual operation");
			return false;
		}

		return false;

	}

	void set_score(roah_rsbb::Score const& score) {
		Time now = Time::now();
		bool group_found = false;

		for (ScoringItem& i : scoring_) {
			if ( (score.group == i.group) && (score.desc == i.desc)) {
				i.current_value = score.value;
				log_.log_score ("/rsbb_log/score", now, score);

				group_found = true;
				break;
			}
		}

		send_referee_score();

		if(!group_found) ROS_ERROR_STREAM("Did not find group " << score.group << " desc " << score.desc);
		return;

	}


	/*************************************************
	 *                                               *
	 *   Benchmark Script communication functions    *
	 *                                               *
	 ************************************************/

	bool execute_manual_operation_callback(ExecuteManualOperation::Request& req, ExecuteManualOperation::Response& res) {
		printStates();

		res.refbox_state = refbox_state_;
		res.result.data = false;

		// BmBox requests a Manual Operation
		if(refbox_state_.manual_operation_state != RefBoxState::READY){
			printStates();
			ROS_ERROR("Requested manual operation with inconsistent states");
			return true;
		}

		// The manual operation is described by the string in the payload of the message
		manual_operation_ = req.request.data;

		set_manual_operation_state(RefBoxState::EXECUTING_MANUAL_OPERATION, req.request.data);

		res.refbox_state = refbox_state_;
		res.result.data = true;

		return true;
	}

	bool execute_goal_callback(ExecuteGoal::Request& req, ExecuteGoal::Response& res) {
		printStates();

		Time now = Time::now();

		res.refbox_state = refbox_state_;
		res.result.data = false;

		if(!(((refbox_state_.goal_execution_state == RefBoxState::READY)
		)
		&&  (refbox_state_.benchmark_state == RefBoxState::EXECUTING_BENCHMARK)
		&&  (state_ == roah_rsbb_msgs::BenchmarkState_State_STOP
		||   state_ == roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT))) {
			printStates();
			ROS_ERROR("Requested goal execution with inconsistent states (RefBox state not READY nor GOAL_TIMEOUT, or benchmark state not STOP nor WAITING_RESULT (from previous goal))");
			return true;
		}

		// set goal payload and goal timeout as specified by the BmBox
		current_goal_payload_ = req.request.data;
		if(req.timeout.data > 0) current_goal_timeout_ = req.timeout.data;

		// start the goal execution
		start_goal_execution();

		if(state_ == roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT){
			cout << "execute_goal_callback: BmBox requests the start of a goal execution (state: WAITING_RESULT) goal: " << req.request.data << endl;
		} else if(state_ == roah_rsbb_msgs::BenchmarkState_State_STOP){
			cout << "execute_goal_callback: BmBox requests the start of a goal execution (state: STOP) goal: " << req.request.data << endl;
		}

		res.refbox_state = refbox_state_;
		res.result.data = true;

		return true;
	}

	/* TO BE IMPLEMENTED Maybe ?
	 * Called by the benchmark script to abort the current goal
	 */
//	bool abort_goal_callback(EndGoal::Request& req, EndGoal::Response& res){
//		cout << "abort_goal_callback" << endl << endl;
//
//		set_refbox_state(Time::now(), RefBoxState::READY);
//
//		return true;
//	}

	/*
	 * Called by the benchmark script when the benchmark is ended and it can be terminated
	 */
	bool end_benchmark_callback(EndBenchmark::Request& req, EndBenchmark::Response& res) {
		printStates();

		cout << endl << endl << "end_benchmark_callback" << endl << endl;

		Time now = Time::now();

		res.refbox_state = refbox_state_;
		res.result.data = false;

		if(!(refbox_state_.benchmark_state == RefBoxState::EXECUTING_BENCHMARK
		&& (refbox_state_.goal_execution_state == RefBoxState::READY
//				|| refbox_state_.goal_execution_state == RefBoxState::GOAL_TIMEOUT
				)
		&& refbox_state_.manual_operation_state == RefBoxState::READY)) {
			printStates();
			ROS_ERROR("Requested end of benchmark with inconsistent states");
			return true;
		}

		// BmBox requests to end the benchmark
		set_benchmark_state(RefBoxState::END); // that means the benchmark has ended naturally, so there is a score and the whole thing can be destroyed

		phase_post("Benchmark complete!");

		res.refbox_state = refbox_state_;
		res.result.data = true;
		return true;
	}

	void bmbox_state_callback(BmBoxState::ConstPtr const& msg) {
		printStates();

		last_bmbox_state_ = msg;

		printStates();

	}

	void bmbox_status_callback(SystemStatus::ConstPtr const& msg) {
		printStates();

		last_bmbox_status_ = *msg;

		printStates();

	}

	void record_server_status_callback(SystemStatus::ConstPtr const& msg) {
		printStates();

		last_record_server_status_ = *msg;

		printStates();

	}



	/*************************************
	 *                                    *
	 *   Robot Communication functions    *
	 *                                    *
	 *************************************/

	void receive_robot_state_2(Time const& now, roah_rsbb_msgs::RobotState const& msg) {
		printStates_robot_state_ = msg.robot_state();
		printStates();

		switch (state_) {
		case roah_rsbb_msgs::BenchmarkState_State_STOP:
			break;


		case roah_rsbb_msgs::BenchmarkState_State_PREPARE:

			if (msg.robot_state() == roah_rsbb_msgs::RobotState_State_WAITING_GOAL) {

				if (refbox_state_.goal_execution_state == RefBoxState::TRANSMITTING_GOAL) {

					cout << "receive_robot_state_2: robot finished preparation, transmitting goal" << endl;

					// Update Benchmark state to GOAL_TX
					set_state(now, roah_rsbb_msgs::BenchmarkState_State_GOAL_TX, "Robot finished preparation, received goal from BmBox, starting execution");
					set_goal_execution_state(RefBoxState::TRANSMITTING_GOAL);

				} else {
					ROS_ERROR("BenchmarkState_State_PREPARE and RobotState_State_WAITING_GOAL and NOT RefBoxState::TRANSMITTING_GOAL");
				}
			}

			break;


		case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:

			if (refbox_state_.goal_execution_state == RefBoxState::TRANSMITTING_GOAL) {

				if (msg.robot_state() == roah_rsbb_msgs::RobotState_State_EXECUTING) {

					cout << "receive_robot_state_2: robot received goal, started execution" << endl;

					GoalStarted goal_execution_started;
					goal_execution_started.request.refbox_state = get_goal_execution_state_copy(RefBoxState::EXECUTING_GOAL);

					if (goal_execution_started_service_.call(goal_execution_started) && goal_execution_started.response.result){
						ROS_INFO("Called bmbox/goal_execution_started service");

						set_state(now, roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT, "Robot received goal, waiting for result");
						set_goal_execution_state(RefBoxState::EXECUTING_GOAL);

					}else{
						ROS_ERROR("Failed to call bmbox/goal_execution_started service or bmbox refused to acknowledge goal execution start");
					}

				}

			} else {
				ROS_ERROR("BenchmarkState_State_GOAL_TX and NOT RefBoxState::TRANSMITTING_GOAL");
			}

			break;


		case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:

			if (!(refbox_state_.goal_execution_state == RefBoxState::EXECUTING_GOAL
			||  refbox_state_.goal_execution_state == RefBoxState::READY
			)){
				ROS_ERROR("BenchmarkState_State_WAITING_RESULT and NOT RefBoxState::EXECUTING_GOAL or RefBoxState::GOAL_TIMEOUT");
			}

			if (refbox_state_.goal_execution_state == RefBoxState::EXECUTING_GOAL
			&&  msg.robot_state() == roah_rsbb_msgs::RobotState_State_RESULT_TX) {

				cout << "receive_robot_state_2: received goal result" << endl;

				end_goal_execution(true, false, msg.has_generic_result()?msg.generic_result():"");

				GoalComplete goal_complete;
				goal_complete.request.goal_result = msg.has_generic_result()?msg.generic_result():"";
				goal_complete.request.goal_timeout = false;
				goal_complete.request.refbox_state = get_goal_execution_state_copy(RefBoxState::READY);

				if (goal_complete_service_.call(goal_complete) && goal_complete.response.result) {
					ROS_INFO("Called bmbox/goal_complete service");
					set_goal_execution_state(RefBoxState::READY, msg.has_generic_result()?msg.generic_result():"");
				}else{
					// TO BE IMPLEMENTED MAYBE repeat call without relying on robot state callback
					ROS_ERROR("Failed to call bmbox/goal_complete service or bmbox refused to acknowledge that goal is completed and receive the result");
				}

			}

			break;
		}


		if (event_.benchmark.commands_devices) {
			if (msg.has_devices_switch_1() && (msg.devices_switch_1() != ss_.last_devices_state->switch_1)) {
				roah_devices::Bool b;
				b.request.data = msg.devices_switch_1();
				call_service("/devices/switch_1/set", b);
				log_.log_uint8("/rsbb_log/devices/switch_1", now, b.request.data ? 1 : 0);
			}
			if (msg.has_devices_switch_2() && (msg.devices_switch_2() != ss_.last_devices_state->switch_2)) {
				roah_devices::Bool b;
				b.request.data = msg.devices_switch_2();
				call_service("/devices/switch_2/set", b);
				log_.log_uint8("/rsbb_log/devices/switch_2", now, b.request.data ? 1 : 0);
			}
			if (msg.has_devices_switch_3() && (msg.devices_switch_3() != ss_.last_devices_state->switch_3)) {
				roah_devices::Bool b;
				b.request.data = msg.devices_switch_3();
				call_service("/devices/switch_3/set", b);
				log_.log_uint8("/rsbb_log/devices/switch_3", now, b.request.data ? 1 : 0);
			}
			if (msg.has_devices_blinds() && (msg.devices_blinds() != ss_.last_devices_state->blinds)) {
				roah_devices::Percentage p;
				p.request.data = msg.devices_blinds();
				call_service("/devices/blinds/set", p);
				log_.log_uint8("/rsbb_log/devices/blinds", now, p.request.data);
			}
			if (msg.has_devices_dimmer() && (msg.devices_dimmer() != ss_.last_devices_state->dimmer)) {
				roah_devices::Percentage p;
				p.request.data = msg.devices_dimmer();
				call_service("/devices/dimmer/set", p);
				log_.log_uint8("/rsbb_log/devices/dimmer", now, p.request.data);
			}
			if (msg.has_tablet_display_map() && (ss_.tablet_display_map != msg.tablet_display_map())) {
				ss_.tablet_display_map = msg.tablet_display_map();
				log_.log_uint8("/rsbb_log/tablet/display_map", now, ss_.tablet_display_map ? 1 : 0);
			}
		}


	}

	string get_bmbox_state_string(BmBoxState::_state_type r){
		string s;

		switch (r) {
		case BmBoxState::COMPLETED_MANUAL_OPERATION:	s = "COMP_MO";	 		break;
		case BmBoxState::END:							s = "END"; 				break;
		case BmBoxState::EXECUTING_GOAL:				s = "EXEC GOAL"; 		break;
		case BmBoxState::READY: 						s = "READY"; 			break;
		case BmBoxState::START: 						s = "STARTING UP";	 	break;
		case BmBoxState::TRANSMITTING_GOAL: 			s = "TX GOAL"; 			break;
		case BmBoxState::TRANSMITTING_SCORE: 			s = "TX SCORE"; 		break;
		case BmBoxState::WAITING_CLIENT: 				s = "WAITING START"; 	break;
		case BmBoxState::WAITING_MANUAL_OPERATION: 		s = "EXEC M.O."; 		break;
		case BmBoxState::WAITING_RESULT: 				s = "WAITING RESULT";	break;
		default:										s = "UNKNOWN";			break;
		}

		return s;
	}


	string get_refbox_state_string(RefBoxState::_benchmark_state_type r){
		string s;

		switch (r) {
		case RefBoxState::START:						s = "WAITING START";	break;
		case RefBoxState::EXECUTING_BENCHMARK:			s = "EXECUTING";		break;
		case RefBoxState::END:							s = "END";				break;
		case RefBoxState::STOP:							s = "STOP";				break;
		case RefBoxState::EMERGENCY_STOP:				s = "EMERGENCY STOP";	break;
		case RefBoxState::ERROR:						s = "ERROR";			break;
		case RefBoxState::GLOBAL_TIMEOUT:				s = "GLOBAL TIMEOUT";	break;
		case RefBoxState::READY:						s = "READY";			break;
		case RefBoxState::TRANSMITTING_GOAL:			s = "TRANSMITTING";		break;
		case RefBoxState::EXECUTING_GOAL:				s = "EXECUTING";		break;
		case RefBoxState::GOAL_TIMEOUT:					s = "GOAL TIMEOUT";		break;
		case RefBoxState::EXECUTING_MANUAL_OPERATION:	s = "EXECUTING";		break;
		default:										s = "UNKNOWN";			break;
		}

		return s;
	}

	string get_robot_state_string(roah_rsbb_msgs::RobotState::State r){
		string s;

		switch (r) {
		case roah_rsbb_msgs::RobotState_State::RobotState_State_PREPARING:			s = "PREPARING"; 	break;
		case roah_rsbb_msgs::RobotState_State::RobotState_State_WAITING_GOAL:		s = "WAITING GOAL"; break;
		case roah_rsbb_msgs::RobotState_State::RobotState_State_EXECUTING:			s = "EXECUTING";	break;
		case roah_rsbb_msgs::RobotState_State::RobotState_State_RESULT_TX:			s = "RESULT TX"; 	break;
		case roah_rsbb_msgs::RobotState_State::RobotState_State_STOP:				s = "STOP"; 		break;
		default:																	s = "UNKNOWN";		break;
		}

		return s;
	}

	void printStates(){

		if(printStates_last_last_bmbox_state_ == last_bmbox_state_->state
		&& printStates_last_benchmark_phase_ == refbox_state_.benchmark_state
		&& printStates_last_goal_execution_state_ == refbox_state_.goal_execution_state
		&& printStates_last_manual_operation_state_ == refbox_state_.manual_operation_state
		&& printStates_last_benchmark_state_ == state_
		&& printStates_last_robot_state_ == printStates_robot_state_
		)	return;

		printStates_last_last_bmbox_state_ = last_bmbox_state_->state;
		printStates_last_benchmark_phase_ = refbox_state_.benchmark_state;
		printStates_last_goal_execution_state_ = refbox_state_.goal_execution_state;
		printStates_last_manual_operation_state_ = refbox_state_.manual_operation_state;
		printStates_last_benchmark_state_ = state_;
		printStates_last_robot_state_ = printStates_robot_state_;

		cout << "\tS  ";
		switch (last_bmbox_state_->state) {
		case BmBoxState::COMPLETED_MANUAL_OPERATION:	cout << "COMPLETED_MO      ";	 	break;
		case BmBoxState::END:							cout << "END               "; 		break;
		case BmBoxState::EXECUTING_GOAL:				cout << "EXECUTING_GOAL    "; 		break;
		case BmBoxState::READY: 						cout << "READY             "; 		break;
		case BmBoxState::START: 						cout << "START             "; 		break;
		case BmBoxState::TRANSMITTING_GOAL: 			cout << "TRANSMITTING_GOAL "; 		break;
		case BmBoxState::TRANSMITTING_SCORE: 			cout << "TRANSMITTING_SCORE"; 		break;
		case BmBoxState::WAITING_CLIENT: 				cout << "WAITING_CLIENT    "; 		break;
		case BmBoxState::WAITING_MANUAL_OPERATION: 		cout << "WAITING_MO        "; 		break;
		case BmBoxState::WAITING_RESULT: 				cout << "WAITING_RESULT    "; 		break;
		default:										cout << last_bmbox_state_->state; 	break;
		}

		cout << "\tBS ";
		switch (refbox_state_.benchmark_state) {
		case RefBoxState::START:						cout << "START              "; 			break;
		case RefBoxState::EXECUTING_BENCHMARK:			cout << "EXECUTING_BENCHMARK"; 			break;
		case RefBoxState::END:							cout << "END                "; 			break;
		case RefBoxState::STOP:							cout << "STOP               "; 			break;
		case RefBoxState::EMERGENCY_STOP:				cout << "EMERGENCY_STOP     "; 			break;
		case RefBoxState::ERROR:						cout << "ERROR              "; 			break;
		case RefBoxState::GLOBAL_TIMEOUT:				cout << "GLOBAL_TIMEOUT     "; 			break;
		case RefBoxState::READY:						cout << "READY              "; 			break;
		case RefBoxState::TRANSMITTING_GOAL:			cout << "TRANSMITTING_GOAL  "; 			break;
		case RefBoxState::EXECUTING_GOAL:				cout << "EXECUTING_GOAL     ";			break;
		case RefBoxState::GOAL_TIMEOUT:					cout << "GOAL_TIMEOUT       "; 			break;
		case RefBoxState::EXECUTING_MANUAL_OPERATION:	cout << "EXECUTING_MO       "; 			break;
		default:										cout << refbox_state_.benchmark_state;	break;
		}

		cout << "\tGS ";
		switch (refbox_state_.goal_execution_state) {
		case RefBoxState::START:						cout << "START              "; 				break;
		case RefBoxState::EXECUTING_BENCHMARK:			cout << "EXECUTING_BENCHMARK"; 				break;
		case RefBoxState::END:							cout << "END                "; 				break;
		case RefBoxState::STOP:							cout << "STOP               "; 				break;
		case RefBoxState::EMERGENCY_STOP:				cout << "EMERGENCY_STOP     "; 				break;
		case RefBoxState::ERROR:						cout << "ERROR              "; 				break;
		case RefBoxState::GLOBAL_TIMEOUT:				cout << "GLOBAL_TIMEOUT     "; 				break;
		case RefBoxState::READY:						cout << "READY              "; 				break;
		case RefBoxState::TRANSMITTING_GOAL:			cout << "TRANSMITTING_GOAL  "; 				break;
		case RefBoxState::EXECUTING_GOAL:				cout << "EXECUTING_GOAL     "; 				break;
		case RefBoxState::GOAL_TIMEOUT:					cout << "GOAL_TIMEOUT       "; 				break;
		case RefBoxState::EXECUTING_MANUAL_OPERATION:	cout << "EXECUTING_MO       "; 				break;
		default:										cout << refbox_state_.goal_execution_state;	break;
		}

		cout << "\tMS ";
		switch (refbox_state_.manual_operation_state) {
		case RefBoxState::START:						cout << "START              "; 					break;
		case RefBoxState::EXECUTING_BENCHMARK:			cout << "EXECUTING_BENCHMARK"; 					break;
		case RefBoxState::END:							cout << "END                "; 					break;
		case RefBoxState::STOP:							cout << "STOP               "; 					break;
		case RefBoxState::EMERGENCY_STOP:				cout << "EMERGENCY_STOP     "; 					break;
		case RefBoxState::ERROR:						cout << "ERROR              "; 					break;
		case RefBoxState::GLOBAL_TIMEOUT:				cout << "GLOBAL_TIMEOUT     "; 					break;
		case RefBoxState::READY:						cout << "READY              "; 					break;
		case RefBoxState::TRANSMITTING_GOAL:			cout << "TRANSMITTING_GOAL  "; 					break;
		case RefBoxState::EXECUTING_GOAL:				cout << "EXECUTING_GOAL     "; 					break;
		case RefBoxState::GOAL_TIMEOUT:					cout << "GOAL_TIMEOUT       "; 					break;
		case RefBoxState::EXECUTING_MANUAL_OPERATION:	cout << "EXECUTING_MO       "; 					break;
		default:										cout << refbox_state_.manual_operation_state;	break;
		}

		cout << "\tRB ";
		switch (state_) {
		case roah_rsbb_msgs::BenchmarkState_State_PREPARE:							cout << "PREPARE       ";	break;
		case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:							cout << "GOAL_TX       "; 	break;
		case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:					cout << "WAITING_RESULT";	break;
		case roah_rsbb_msgs::BenchmarkState_State_STOP:								cout << "STOP          "; 	break;
		default:																	cout << state_;				break;
		}

		cout << "\tR  ";
		switch (printStates_robot_state_) {
		case roah_rsbb_msgs::RobotState_State::RobotState_State_PREPARING:			cout << "PREPARING"; 	break;
		case roah_rsbb_msgs::RobotState_State::RobotState_State_WAITING_GOAL:		cout << "WAITING_GOAL"; break;
		case roah_rsbb_msgs::RobotState_State::RobotState_State_EXECUTING:			cout << "EXECUTING";	break;
		case roah_rsbb_msgs::RobotState_State::RobotState_State_RESULT_TX:			cout << "RESULT_TX"; 	break;
		case roah_rsbb_msgs::RobotState_State::RobotState_State_STOP:				cout << "STOP"; 		break;
		default:																	cout << state_;			break;
		}

		cout << endl;

	}

};








class ExecutingAllRobotsBenchmark: public ExecutingBenchmark {
	vector<Event> dummy_events_;
	vector<unique_ptr<ExecutingSimpleBenchmark>> simple_benchmarks_;

	void phase_exec_2(Time const& now) {
		for (auto const& i : simple_benchmarks_) {
			i->start();
		}
		set_state(now, roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT, "Preparing and executing");
	}

	void phase_post_2(Time const& now) {
for	(auto const& i : simple_benchmarks_) {
		i->stop();
	}
}

static void
end()
{
	// empty
}

public:
ExecutingAllRobotsBenchmark (CoreSharedState& ss,
		Event const& event,
		boost::function<void() > end)
: ExecutingBenchmark (ss, event, end)
{
	for (roah_rsbb::RobotInfo const& ri : ss_.active_robots.get ()) {
		if (ss_.benchmarking_robots.count (ri.team)) {
			ROS_ERROR_STREAM ("Ignoring robot of team " << ri.team << " because it is already executing a benchmark");
			continue;
		}

		dummy_events_.push_back (event);
		dummy_events_.back().team = ri.team;
		dummy_events_.back().password = ss_.passwords.get (dummy_events_.back().team);

		bool ok = false;
		do {
			try {
				simple_benchmarks_.push_back (unique_ptr<ExecutingSimpleBenchmark> (new ExecutingSimpleBenchmark (ss, dummy_events_.back(), &ExecutingAllRobotsBenchmark::end, ri.robot, false)));
				ok = true;
			}
			catch (const std::exception& exc) {
				std::cerr << exc.what();
				ROS_ERROR_STREAM ("Failed to create a private channel. Retrying on next port.");
			}
		}
		while (! ok);
	}
}

void
fill_2 (Time const& now,
		roah_rsbb::ZoneState& zone)
{
	unsigned prep = 0, exec = 0, stopped = 0;
	for (auto const& i : simple_benchmarks_) {
		switch (i->state()) {
			case roah_rsbb_msgs::BenchmarkState_State_STOP:
			++stopped;
			break;
			case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
			case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
			++prep;
			break;
			case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
			++exec;
			break;
		}
	}

	add_to_sting (zone.state) << "Robots preparing: " << prep;
	add_to_sting (zone.state) << "Robots executing: " << exec;
	add_to_sting (zone.state) << "Robots stopped: " << stopped;
}

void
stop_communication()
{
	for (auto const& i : simple_benchmarks_) {
		i->stop_communication();
	}
}
};

#endif
