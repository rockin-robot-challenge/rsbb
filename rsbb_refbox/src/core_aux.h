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

#ifndef __CORE_AUX_H__
#define __CORE_AUX_H__

#include "core_includes.h"

class add_to_sting: public ostringstream {
	string& s_;

public:
	add_to_sting(string& s) :
			ostringstream(), s_(s) {
		(*this) << s;
		if (!s.empty()) {
			(*this) << endl;
		}
	}

	~add_to_sting() {
		s_ = str();
	}
};

void abort_rsbb() {
	if (service::waitForService("roah_rsbb_shutdown", 500)) {
		std_srvs::Empty s;
		if (!service::call("roah_rsbb_shutdown", s)) {
			ROS_ERROR("Error calling roah_rsbb_shutdown service");
		}
	} else {
		ROS_ERROR("Could not find roah_rsbb_shutdown service");
	}

	shutdown();

	abort();
}

template<typename T> T yamlschedget(YAML::Node const& node, string const& key) {
	if (!node[key]) {
		ROS_FATAL_STREAM("Schedule file is missing a \"" << key << "\" entry!");
		abort_rsbb();
	}
	return node[key].as<T>();
}

string to_string(Time const& time) {
	return boost::posix_time::to_simple_string(boost::date_time::c_local_adjustor<boost::posix_time::ptime>::utc_to_local(time.toBoost()));
}

#endif
