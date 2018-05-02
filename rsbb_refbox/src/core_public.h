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

#ifndef __CORE_PUBLIC_H__
#define __CORE_PUBLIC_H__

#include "core_includes.h"

#include "core_shared_state.h"
#include "core_public_channel.h"
#include "core_zone_manager.h"

class CorePublic: boost::noncopyable {
	CoreSharedState& ss_;
	CoreZoneManager& zone_manager_;

	Publisher pub_;
	Timer pub_timer_;

	Time relevant_time_;

	void transmit(const TimerEvent& = TimerEvent()) {
		Time now = Time::now();

		auto msg = boost::make_shared<roah_rsbb::CoreToPublic>();
		msg->clock = to_string(Time(now.sec, 0));

		multimap<Time, roah_rsbb::ScheduleInfo> map;
		zone_manager_.msg(now, map);
		for (auto const& i : map) {
			if (i.second.running) {
				relevant_time_ = i.first;
				break;
			}
		}
		{
			multimap<Time, roah_rsbb::ScheduleInfo>::iterator i = map.begin();
			while (i != map.end()) {
				if (i->first < relevant_time_) {
					i = map.erase(i);
				} else {
					break;
				}
			}
		}
		for (auto const& i : map) {
			msg->schedule.push_back(i.second);
		}

		pub_.publish(msg);
	}

public:
	CorePublic(CoreSharedState& ss, CoreZoneManager& zone_manager) :
			ss_(ss), zone_manager_(zone_manager), pub_(ss_.nh.advertise<roah_rsbb::CoreToPublic>("/core/to_public", 1, true)), pub_timer_(
					ss_.nh.createTimer(Duration(0.5), &CorePublic::transmit, this)), relevant_time_(TIME_MIN) {
		transmit();
	}
};

#endif
