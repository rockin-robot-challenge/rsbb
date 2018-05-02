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

#ifndef __CORE_SHARED_STATE_H__
#define __CORE_SHARED_STATE_H__

#include "core_includes.h"

#include "core_aux.h"

class ActiveRobots: boost::noncopyable {
	Duration robot_timeout_;

	// sum . map size team_robot_map_ == size last_beacon_map_
	map<string, map<string, roah_rsbb::RobotInfo::ConstPtr>> team_robot_map_;
	map<Time, roah_rsbb::RobotInfo::ConstPtr> last_beacon_map_;

	void update() {
		auto now = Time::now();

		while ((!last_beacon_map_.empty()) && ((last_beacon_map_.begin()->first + robot_timeout_) < now)) {
			team_robot_map_[last_beacon_map_.begin()->second->team].erase(last_beacon_map_.begin()->second->robot);
			last_beacon_map_.erase(last_beacon_map_.begin());
		}
	}

public:
	ActiveRobots() :
			robot_timeout_(param_direct<double>("~robot_timeout", 30.0)) {
	}

	void add(roah_rsbb::RobotInfo::ConstPtr const& ri) {
		auto last_team = team_robot_map_.find(ri->team);
		if (last_team != team_robot_map_.end()) {
			auto last = last_team->second.find(ri->robot);
			if (last != last_team->second.end()) {
				last_beacon_map_.erase(last->second->beacon);
				last_beacon_map_[ri->beacon] = ri;
				last->second = ri;
				return;
			}
		}
		team_robot_map_[ri->team][ri->robot] = ri;
		last_beacon_map_[ri->beacon] = ri;
	}

	void add(string const& team, string const& robot, Duration const& skew, Time const& beacon) {
		auto msg = boost::make_shared<roah_rsbb::RobotInfo>();
		msg->team = team;
		msg->robot = robot;
		msg->skew = skew;
		msg->beacon = beacon;
		add(msg);
	}

	void msg(vector<roah_rsbb::RobotInfo>& msg) {
		update();

		for (auto const& iteam : team_robot_map_) {
			for (auto const& i : iteam.second) {
				msg.push_back(*(i.second));
			}
		}
	}

	vector<roah_rsbb::RobotInfo> get() {
		update();

		vector<roah_rsbb::RobotInfo> ret;
		ret.reserve(team_robot_map_.size());

		for (auto const& iteam : team_robot_map_) {
			map<string, roah_rsbb::RobotInfo::ConstPtr> rm = iteam.second;

			if (!rm.empty()) {
				ret.push_back(*(rm.begin()->second));
			}
		}

		return ret;
	}

	roah_rsbb::RobotInfo get(string const& team) {
		update();

		map<string, roah_rsbb::RobotInfo::ConstPtr> rm = team_robot_map_[team];

		if (rm.empty()) {
			return roah_rsbb::RobotInfo();
		}

		return *(rm.begin()->second);
	}
};

struct ScoringItem {
	typedef enum {
		SCORING_BOOL, SCORING_UINT
	} scoring_type_t;

	string group;
	string desc;
	scoring_type_t type;
	int32_t current_value;

	ScoringItem(string const& benchmark, string const& group_name, YAML::Node const& item_node) :
			group(group_name), current_value(0) {
		using namespace YAML;

		if (!item_node["type"]) {
			ROS_FATAL_STREAM("Benchmark \"" << benchmark << "\" scoring item in \"" << group_name << "\" is missing a \"type\" entry! :\n" << item_node);
			abort_rsbb();
		}
		string type_s = item_node["type"].as<string>();
		if (type_s == "bool") {
			type = SCORING_BOOL;
		} else if (type_s == "uint") {
			type = SCORING_UINT;
		} else {
			ROS_FATAL_STREAM("Benchmark \"" << benchmark << "\" scoring item in \"" << group_name << "\" type is unknown:" << type_s);
			abort_rsbb();
		}

		if (!item_node["desc"]) {
			ROS_FATAL_STREAM("Benchmark \"" << benchmark << "\" scoring item in \"" << group_name << "\" is missing a \"desc\" entry! :\n" << item_node);
			abort_rsbb();
		}
		desc = item_node["desc"].as<string>();
	}

	YAML::Node to_yaml_node() {
		using namespace YAML;

		YAML::Node item_node;
		item_node["group_name"] = group;
		item_node["desc"] = desc;
		item_node["current_value"] = current_value;
		item_node["type"] = type == SCORING_BOOL ? "bool" : "uint";

		return item_node;
	}
};

struct Benchmark {
	string code;
	string desc;
	string name;
	int order;
	bool scripted;
	bool multiple_robots;
	bool commands_devices;
	Duration timeout;
	Duration total_timeout;
	vector<string> record_topics;
	vector<ScoringItem> scoring;
};

class Benchmarks {
	map<string, Benchmark> by_code_;
	map<int, string> codes_by_order_;

public:
	Benchmarks() {
		using namespace YAML;

		Node file = LoadFile(param_direct<string>("~benchmarks_file", "benchmarks.yaml"));

		Node benchmarks_description_node = file["benchmarks_description"];

		if (!benchmarks_description_node || !benchmarks_description_node.IsSequence()) {
			ROS_FATAL_STREAM("In the benchmarks description file, benchmarks_description should be the root entry and should be a sequence of benchmark entries");
			abort_rsbb();
		}

		for (Node const& benchmark_node : benchmarks_description_node) {
			if (!benchmark_node.IsMap()) {
				ROS_FATAL_STREAM("In the benchmarks description file, a benchmark entry is not a map");
				abort_rsbb();
			}
			if (!benchmark_node["code"]) {
				ROS_FATAL_STREAM("In benchmarks_description, a \"code\" [string] entry is missing");
				abort_rsbb();
			}
			if (!benchmark_node["order"]) {
				ROS_FATAL_STREAM("In benchmarks_description, \"order\" is missing for benchmark [" << benchmark_node["code"] << "]");
				ROS_FATAL_STREAM("In benchmarks_description, for each benchmark a \"order\" [int] entry should be specified.");
				abort_rsbb();
			}
			if (!benchmark_node["desc"]) {
				ROS_FATAL_STREAM("Benchmarks file is missing a \"desc\" entry for benchmark [" << benchmark_node["code"] << "]");
				ROS_FATAL_STREAM("In benchmarks_description, for each benchmark a \"desc\" [string] entry should be specified");
				abort_rsbb();
			}
			if (!benchmark_node["scripted"]) {
				ROS_FATAL_STREAM("In benchmarks_description, \"scripted\" is missing for benchmark [" << benchmark_node["code"] << "]");
				ROS_FATAL_STREAM("In benchmarks_description, for each benchmark a \"scripted\" [bool] entry should be specified.");
				abort_rsbb();
			}
			if (!benchmark_node["multiple_robots"]) {
				ROS_FATAL_STREAM("In benchmarks_description, \"multiple_robots\" is missing for benchmark [" << benchmark_node["code"] << "]");
				ROS_FATAL_STREAM("In benchmarks_description, for each benchmark a \"multiple_robots\" [bool] entry should be specified.");
				abort_rsbb();
			}
			if (!benchmark_node["commands_devices"]) {
				ROS_FATAL_STREAM("In benchmarks_description, \"commands_devices\" is missing for benchmark [" << benchmark_node["code"] << "]");
				ROS_FATAL_STREAM("In benchmarks_description, for each benchmark a \"commands_devices\" [bool] entry should be specified.");
				abort_rsbb();
			}
			if (benchmark_node["record_topics"] && !benchmark_node["record_topics"].IsSequence()) {
				ROS_FATAL_STREAM("In benchmarks_description, \"record_topics\" is not a sequence for benchmark [" << benchmark_node["code"] << "]");
				ROS_FATAL_STREAM("In benchmarks_description, the \"record_topics\" entry should be a sequence");
				abort_rsbb();
			}
			if (benchmark_node["timeout"]){

				if(benchmark_node["global_timeout"] || benchmark_node["default_goal_timeout"]){
					ROS_FATAL_STREAM("In benchmarks_description, both \"timeout\" and \"global_timeout\" or \"default_goal_timeout\" are present for benchmark [" << benchmark_node["code"] << "]");
					ROS_FATAL_STREAM("In benchmarks_description, for each benchmark either \"timeout\" (when scripted = false) or both \"global_timeout\" and \"default_goal_timeout\" (when scripted = true) entries should be specified");
					abort_rsbb();
				}

			}else{
				if(!benchmark_node["global_timeout"] || !benchmark_node["default_goal_timeout"]){
					ROS_FATAL_STREAM("In benchmarks_description, nor \"timeout\" nor \"global_timeout\" and \"default_goal_timeout\" are present for benchmark [" << benchmark_node["code"] << "]");
					ROS_FATAL_STREAM("In benchmarks_description, for each benchmark either \"timeout\" (when scripted = false) or both \"global_timeout\" and \"default_goal_timeout\" (when scripted = true) entries should be specified");
					abort_rsbb();
				}
			}

			Benchmark b;
			b.code = benchmark_node["code"].as<string>();
			b.order = benchmark_node["order"].as<int>();
			b.name = b.code; //benchmark_node["name"].as<string>();
			b.desc = benchmark_node["desc"].as<string>();
			b.scripted = benchmark_node["scripted"].as<bool>();
			b.multiple_robots = benchmark_node["multiple_robots"].as<bool>();
			b.commands_devices = benchmark_node["commands_devices"].as<bool>();

			if(benchmark_node["timeout"]){
				b.timeout = Duration(benchmark_node["timeout"].as<double>());
				b.total_timeout = b.timeout;
			}else{
				b.timeout = Duration(benchmark_node["default_goal_timeout"].as<double>());
				b.total_timeout = Duration(benchmark_node["global_timeout"].as<double>());
			}

			if (benchmark_node["scoring"]) {

				if (!benchmark_node["scoring"].IsSequence()) {
					ROS_FATAL_STREAM("In benchmarks_description, the \"scoring\" entry is not a sequence for benchmark [" << benchmark_node["code"] << "]");
					abort_rsbb();
				}

				for (Node const& scoring_node : benchmark_node["scoring"]) {

					if (!scoring_node.IsMap()) {
						ROS_FATAL_STREAM("In benchmarks_description, the \"scoring\" entry is not a sequence of maps for benchmark [" << benchmark_node["code"] << "]");
						abort_rsbb();
					}

					for (YAML::const_iterator it = scoring_node.begin(); it != scoring_node.end(); ++it) {

						string group_name = it->first.as<string>();

						if (!it->second.IsSequence()) {
							ROS_FATAL_STREAM("In benchmarks_description, the \"scoring \\ " << it->first << "\" entry is not a sequence of maps for benchmark [" << benchmark_node["code"] << "]");
							abort_rsbb();
						}

						for (Node const& item_node : it->second) {
							b.scoring.push_back(ScoringItem(b.name, group_name, item_node));
						}

					}
				}
			}

			if (benchmark_node["record_topics"]) {
				for (Node const& topic_node : benchmark_node["record_topics"]) {
					b.record_topics.push_back(topic_node.as<string>());
				}
			}

			if(codes_by_order_.find(b.order) != codes_by_order_.end()){
				ROS_FATAL_STREAM("In benchmarks_description, duplicated \"order\" entries are present");
				abort_rsbb();
			}

			codes_by_order_[b.order] = b.code;
			by_code_[b.code] = b;

		}
	}

	Benchmark const& get(string const& code) const {
		auto b = by_code_.find(code);
		if (b == by_code_.end()) {
			ROS_FATAL_STREAM("Could not find benchmark with code \"" << code << "\"");
			abort_rsbb();
		}
		return b->second;
	}

	vector<string> get_benchmark_codes() const {
		vector<string> benchmark_codes;

		for(auto benchmark_code_pair : codes_by_order_){
			benchmark_codes.push_back(benchmark_code_pair.second);
		}

		return benchmark_codes;
	}

};

class Passwords {
	map<string, string> passwords_;

public:
	Passwords() {
		using namespace YAML;

		if( param_direct<bool>("~generate_schedule", true)){

			string teams_list_file_path = param_direct<string>("~teams_list_file", "");

			if(teams_list_file_path.empty()){
				ROS_FATAL_STREAM("Teams list file is missing");
				abort_rsbb();
			}

			Node file = LoadFile(teams_list_file_path);

			Node teams_list_node = file["teams_list"];

			if (!teams_list_node || !teams_list_node.IsSequence()) {
				ROS_FATAL_STREAM("teams_list in teams list file is not a sequence");
				abort_rsbb();
			}

			for (auto const& team_node : teams_list_node) passwords_[team_node["name"].as<string>()] = team_node["password"].as<string>();

		}else{

			string passwords_file_path = param_direct<string>("~passwords_file", "");

			if(passwords_file_path.empty()){
				ROS_FATAL_STREAM("Passwords file is missing");
				abort_rsbb();
			}

			Node file = LoadFile(passwords_file_path);

			if (!file.IsMap()) {
				ROS_FATAL_STREAM("Passwords file is not a map");
				abort_rsbb();
			}

			for (auto const& team_node : file) {
				passwords_[team_node.first.as<string>()] = team_node.second.as<string>();
			}
		}
	}

	string const& get(string const& team) const {
		auto b = passwords_.find(team);
		if (b == passwords_.end()) {
			ROS_FATAL_STREAM("Could not find password for team \"" << team << "\"");
			abort_rsbb();
		}
		return b->second;
	}
};

struct CoreSharedState: boost::noncopyable {
	NodeHandle nh;
	ActiveRobots active_robots;
	string status;
	const Benchmarks benchmarks;
	const Passwords passwords;
	const string run_uuid;
	map<string, pair<string, uint32_t>> benchmarking_robots;
	bool tablet_display_map;
	roah_devices::DevicesState::ConstPtr last_devices_state;
	Time last_tablet_time;
	std::shared_ptr<const roah_rsbb_msgs::TabletBeacon> last_tablet;

	unsigned short private_port_;

	CoreSharedState() :
			status("Initializing..."), run_uuid(to_string(boost::uuids::random_generator()())), tablet_display_map(false), last_devices_state(
					boost::make_shared<roah_devices::DevicesState>()), last_tablet_time(TIME_MIN), last_tablet(/*empty*/), private_port_(param_direct<int>("~rsbb_port", 6666)) {
	}

	unsigned short private_port() {
		return ++private_port_;
	}
};

#endif
