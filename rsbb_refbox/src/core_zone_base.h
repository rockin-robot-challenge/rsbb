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

#ifndef __CORE_ZONE_BASE_H__
#define __CORE_ZONE_BASE_H__

#include "core_includes.h"

#include "core_shared_state.h"

struct Event {
	Benchmark benchmark;
	string benchmark_code;
	string team;
	string password;
	unsigned round;
	unsigned run;
	Time scheduled_time;

	// Duration interval_time;

	Event(YAML::Node const& event_node) {
		benchmark_code = yamlschedget<string>(event_node, "benchmark");
		team = yamlschedget<string>(event_node, "team");
		round = yamlschedget<unsigned>(event_node, "round");
//		run = yamlschedget<unsigned>(event_node, "run");
		run = 0;
		scheduled_time = Time::fromBoost(boost::posix_time::time_from_string(yamlschedget<string>(event_node, "scheduled_time")));
		// interval_time = Duration (yamlschedget<double> (event_node, "interval_time"));
	}

	Event(const Benchmark& benchmark_, string team_, string password_, Time scheduled_time_) {
		benchmark = benchmark_;
		benchmark_code = benchmark_.code;
		team = team_;
		round = 0;
		run = 0;
		password = password_;
		scheduled_time = scheduled_time_;
	}

};

std::ostream& operator<<(std::ostream& os, const Event& e) {
	os 		<< "benchmark_code: " << e.benchmark_code
			<< ", team: " << e.team
			<< ", password: " << e.password
			<< ", round: " << e.round
			<< ", run: " << e.run
			<< ", scheduled_time: " << e.scheduled_time;
	return os;
}

class DisplayText: boost::noncopyable {
	ostringstream text_;
	string last_;

public:
	DisplayText() {
	}

	void add(Time const& now, string const& msg) {
		if (msg == last_) {
			return;
		}

		last_ = msg;

		text_ << endl;
		text_ << " - " << to_string(now);
		//text_ << endl;
		text_ << " - ";
		text_ << msg;
	}

	void add(string const& msg) {
		add(Time::now(), msg);
	}

	string str() {
		return text_.str();
	}

	string last(size_t length = 1) {
		string ret = text_.str();
		if (length >= ret.size()) {
			return ret;
		}
		return ret.substr(ret.size() - length);
	}
};

class RsbbLog: boost::noncopyable {
	rosbag::Bag bag_;
	DisplayText& display_text_;

public:
	RsbbLog(string const& team, unsigned round, unsigned run, string const& uuid, DisplayText& display_text) :
			display_text_(display_text) {
		string log_dir = param_direct<string>("~log_dir", ".");
		system(string("mkdir -p " + log_dir).c_str());

		ostringstream o;
		o << log_dir << "/online_log_";
		o << to_string(Time::now());
		o << "_" << team << "_round" << round << "_run" << run;
		o << "_" << uuid << ".bag";
		bag_.open(o.str(), rosbag::bagmode::Write);
	}

	~RsbbLog() {
		bag_.close();
	}

	void log_empty(string const& topic, Time const& time) {
		std_msgs::Empty msg;
		bag_.write(topic, time, msg);

		display_text_.add(time, topic);
	}

	void log_uint8(string const& topic, Time const& time, uint8_t i) {
		std_msgs::UInt8 msg;
		msg.data = i;
		bag_.write(topic, time, msg);

		display_text_.add(time, topic + "\n" + to_string(i));
	}

	void log_string(string const& topic, Time const& time, string const& s) {
		std_msgs::String msg;
		msg.data = s;
		bag_.write(topic, time, msg);

		display_text_.add(time, topic + "\n" + s);
	}

	void log_score(string const& topic, Time const& time, roah_rsbb::Score const& msg) {
		bag_.write(topic, time, msg);

		display_text_.add(time, topic + "\n" + msg.group + ", " + msg.desc + " -> " + to_string(msg.value));
	}

	void set_state(Time const& now, roah_rsbb_msgs::BenchmarkState::State const& state, string const& desc) {
		string state_name;
		switch (state) {
		case roah_rsbb_msgs::BenchmarkState_State_STOP:
			state_name = "BenchmarkState_State_STOP";
			break;
		case roah_rsbb_msgs::BenchmarkState_State_PREPARE:
			state_name = "BenchmarkState_State_PREPARE";
			break;
		case roah_rsbb_msgs::BenchmarkState_State_GOAL_TX:
			state_name = "BenchmarkState_State_GOAL_TX";
			break;
		case roah_rsbb_msgs::BenchmarkState_State_WAITING_RESULT:
			state_name = "BenchmarkState_State_WAITING_RESULT";
			break;
		}

		log_uint8("/rsbb_log/rsbb_state", now, state);
		log_string("/rsbb_log/rsbb_state_str", now, state_name);
		log_string("/rsbb_log/rsbb_state_desc", now, desc);
	}

	void end() {
		log_empty("/rsbb_log/end", Time::now());
	}

#if 0
	template<class T>
	void write (std::string const& topic, ros::Time const& time, T const& msg,
			boost::shared_ptr<ros::M_string> connection_header = boost::shared_ptr<ros::M_string>())
	{
		bag_.write<T> (topic, time, msg, connection_header);
	}
#endif
};

class ReceiverRepeated {
	set<string> last_;
	RsbbLog& log_;
	string topic_;
	DisplayText& display_text_;

public:
	ReceiverRepeated(RsbbLog& log, string const& topic, DisplayText& display_text) :
			log_(log), topic_(topic), display_text_(display_text) {
	}

	void receive(Time const& now, ::google::protobuf::RepeatedPtrField<string> const& field) {
		set<string> this_set;
		for (string const& s : field) {
			this_set.insert(s);
			if (0 == last_.count(s)) {
				display_text_.add(now, topic_ + "\n" + s);
				log_.log_string(topic_, now, s);
			}
		}
		last_ = move(this_set);
	}
};

class TimeControl {
	CoreSharedState& ss_;
	Duration timeout_;

	Time start_time_;

	Duration delay_acc_;bool paused_;
	Time pause_start_;

	Timer timeout_timer_;
	const function<void(void)> timeout_2_;

	void timeout(const TimerEvent& timer_event) {
		if (paused_) {
			return;
		}

		if (start_timer(timer_event.current_real)) {
			return;
		}

		timeout_2_();
	}

	bool start_timer(Time const& now) {
		Duration until_timeout = get_until_timeout(now);
		timeout_timer_.stop();
		if (until_timeout > Duration()) {
			timeout_timer_ = ss_.nh.createTimer(until_timeout, &TimeControl::timeout, this, true, true);
			return true;
		}
		return false;
	}

public:
	TimeControl(CoreSharedState& ss, Duration timeout, function<void(void)> const& timeout_2) :
			ss_(ss), timeout_(timeout), delay_acc_(), paused_(false), timeout_timer_(), timeout_2_(timeout_2) {
	}
	TimeControl(CoreSharedState& ss, Duration timeout, bool paused, function<void(void)> const& timeout_2) :
			ss_(ss), timeout_(timeout), delay_acc_(), paused_(paused), timeout_timer_(), timeout_2_(timeout_2) {
	}

	~TimeControl() {
		timeout_timer_.stop();
	}

	void start_reset(Time const& now) {
		start_time_ = now;
		delay_acc_ = Duration();
		paused_ = false;
		start_timer(now);
	}

	void start_reset(Time const& now, Duration const& new_timeout) {
		timeout_ = new_timeout;
		start_reset(now);
	}

	void stop_pause(Time const& now) {
		if (paused_) {
			return;
		}

		paused_ = true;
		pause_start_ = now;
		timeout_timer_.stop();
	}

	void resume(Time const& now) {
		if (!paused_) {
			return;
		}

		delay_acc_ += (now - pause_start_);
		paused_ = false;
		start_timer(now);
	}

	void resume_hot(Time const& now) {
		if (!paused_) {
			return;
		}

		// Does not accumulate, stopped time actually counts in resets
		paused_ = false;
		start_timer(now);
	}

	Duration get_until_timeout(Time const& now) {
		return start_time_ + timeout_ + delay_acc_ - (paused_ ? pause_start_ : now);
	}

	Duration get_until_timeout_for_timeout(Time const& now, Duration const& timeout) {
		return start_time_ + timeout + delay_acc_ - (paused_ ? pause_start_ : now);
	}

	Duration get_elapsed(Time const& now) {
		return (paused_ ? pause_start_ : now) - start_time_ - delay_acc_;
	}
};

#endif
