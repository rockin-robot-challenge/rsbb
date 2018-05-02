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

#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <ros_roah_rsbb.h>

using namespace std;
using namespace ros;

void bell(std_msgs::Empty::ConstPtr const& /*msg*/) {
	system(param_direct<string>("~bell_ring_command", "mplayer bell.mp3").c_str());
}

void timeout(std_msgs::Empty::ConstPtr const& /*msg*/) {
	system(param_direct<string>("~timeout_ring_command", "mplayer timeout.mp3").c_str());
}

int main(int argc, char** argv) {
	init(argc, argv, "roah_rsbb_sounds");
	NodeHandle nh;

	Subscriber bell_sub = nh.subscribe("/devices/bell", 1, bell);
	Subscriber timeout_sub = nh.subscribe("/timeout", 1, timeout);
	spin();

	return 0;
}
