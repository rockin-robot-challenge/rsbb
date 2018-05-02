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

#include "core_includes.h"

#include "core_shared_state.h"
#include "core_public_channel.h"
#include "core_zone_manager.h"
#include "core_gui.h"
#include "core_public.h"

namespace roah_rsbb {
class Core {
	CoreSharedState ss_;
	CorePublicChannel public_channel_;
	CoreZoneManager zone_manager_;
	CoreGui gui_;
	CorePublic public_;

	Subscriber devices_sub_;

	void devices_callback(roah_devices::DevicesState::ConstPtr const& msg) {
		ss_.last_devices_state = msg;
	}

public:
	Core() :
			ss_(), public_channel_(ss_), zone_manager_(ss_), gui_(ss_, public_channel_, zone_manager_), public_(ss_, zone_manager_), devices_sub_(
					ss_.nh.subscribe("/devices/state", 1, &Core::devices_callback, this)) {
	}
};
}

int main(int argc, char* argv[]) {
	init(argc, argv, "roah_rsbb_core");

	roah_rsbb::Core node;

	spin();

	return 0;
}
