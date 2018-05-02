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

#ifndef __CORE_PUBLIC_CHANNEL_H__
#define __CORE_PUBLIC_CHANNEL_H__

#include "core_includes.h"

#include "core_shared_state.h"

class CorePublicChannel: boost::noncopyable, public roah_rsbb::RosPublicChannel {
	CoreSharedState& ss_;

	Timer beacon_timer_;

	void transmit_beacon(const TimerEvent& = TimerEvent()) {
		ROS_DEBUG("Transmitting beacon");

		roah_rsbb_msgs::RoahRsbbBeacon msg;
		for (auto const& i : ss_.benchmarking_robots) {
			roah_rsbb_msgs::BenchmarkingTeam* bt = msg.add_benchmarking_teams();
			bt->set_team_name(i.first);
			bt->set_robot_name(i.second.first);
			bt->set_rsbb_port(i.second.second);
		}

		msg.mutable_devices_bell()->set_sec(ss_.last_devices_state->bell.sec);
		msg.mutable_devices_bell()->set_nsec(ss_.last_devices_state->bell.nsec);
		msg.set_devices_switch_1(ss_.last_devices_state->switch_1 != 0);
		msg.set_devices_switch_2(ss_.last_devices_state->switch_2 != 0);
		msg.set_devices_switch_3(ss_.last_devices_state->switch_3 != 0);
		msg.set_devices_dimmer(static_cast<uint32_t>(ss_.last_devices_state->dimmer));
		msg.set_devices_blinds(static_cast<uint32_t>(ss_.last_devices_state->blinds));
		msg.set_devices_door_win_detect(ss_.last_devices_state->door_win_detect != 0);

		msg.set_tablet_display_map(ss_.tablet_display_map);
		if (ss_.last_tablet) {
			(*(msg.mutable_tablet_call_time())) = ss_.last_tablet->last_call();
			(*(msg.mutable_tablet_position_time())) = ss_.last_tablet->last_pos();
			msg.set_tablet_position_x(ss_.last_tablet->x());
			msg.set_tablet_position_y(ss_.last_tablet->y());
		} else {
			msg.mutable_tablet_call_time()->set_sec(0);
			msg.mutable_tablet_call_time()->set_nsec(0);
			msg.mutable_tablet_position_time()->set_sec(0);
			msg.mutable_tablet_position_time()->set_nsec(0);
			msg.set_tablet_position_x(0);
			msg.set_tablet_position_y(0);
		}
		send(msg);
	}
	void setup_transmit_beacon(const TimerEvent&) {
		transmit_beacon();

		beacon_timer_.stop();
		beacon_timer_ = ss_.nh.createTimer(Duration(1, 0), &CorePublicChannel::transmit_beacon, this);

		ss_.status = "OK";
	}

	void receive_rsbb_beacon(boost::asio::ip::udp::endpoint endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<const roah_rsbb_msgs::RoahRsbbBeacon> rsbb_beacon) {
		ROS_FATAL_STREAM("Another RSBB running at " << endpoint.address().to_string() << ":" << endpoint.port());

		abort_rsbb();
	}

	void receive_robot_beacon(boost::asio::ip::udp::endpoint endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<const roah_rsbb_msgs::RobotBeacon> msg) {
		Time now = Time::now();
		Time msg_time(msg->time().sec(), msg->time().nsec());
		Duration skew = msg_time - now;

		ROS_DEBUG_STREAM(
				"Received RobotBeacon from " << endpoint.address().to_string() << ":" << endpoint.port() << ", COMP_ID " << comp_id << ", MSG_TYPE " << msg_type << ", team_name: " << msg->team_name() << ", robot_name: " << msg->robot_name() << ", time: " << msg->time().sec() << "." << msg->time().nsec() << ", skew: " << skew);

		ss_.active_robots.add(msg->team_name(), msg->robot_name(), skew, now);
	}

	void receive_tablet_beacon(boost::asio::ip::udp::endpoint endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<const roah_rsbb_msgs::TabletBeacon> msg) {
		ROS_DEBUG_STREAM("Received TabletBeacon from " << endpoint.address().to_string() << ":" << endpoint.port() << ", COMP_ID " << comp_id << ", MSG_TYPE " << msg_type);

		ss_.last_tablet_time = Time::now();
		ss_.last_tablet = msg;
	}

public:
	CorePublicChannel(CoreSharedState& ss) :
			roah_rsbb::RosPublicChannel(param_direct<string>("~rsbb_broadcast_address", "10.0.0.255"), param_direct<int>("~rsbb_port", 6666)), ss_(ss), beacon_timer_(
					ss_.nh.createTimer(Duration(5, 0), &CorePublicChannel::setup_transmit_beacon, this, true)) {
		set_rsbb_beacon_callback(&CorePublicChannel::receive_rsbb_beacon, this);
		set_robot_beacon_callback(&CorePublicChannel::receive_robot_beacon, this);
		set_tablet_beacon_callback(&CorePublicChannel::receive_tablet_beacon, this);

		ROS_INFO("Listening only... beacon transmission will start in 5 seconds.");
	}

	~CorePublicChannel() {
		beacon_timer_.stop();
		signal_rsbb_beacon_received().disconnect_all_slots();
		signal_robot_beacon_received().disconnect_all_slots();
		signal_tablet_beacon_received().disconnect_all_slots();
	}
};

#endif
