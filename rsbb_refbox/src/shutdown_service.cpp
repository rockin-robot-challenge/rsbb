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

#include "ros/ros.h"
#include "std_srvs/Empty.h"

using namespace std;
using namespace ros;

bool shutdown_handler(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
	requestShutdown();
	return true;
}

int main(int argc, char** argv) {
	init(argc, argv, "roah_rsbb_shutdown");
	NodeHandle nh;

	ServiceServer service = nh.advertiseService("roah_rsbb_shutdown", shutdown_handler);
	spin();

	return 0;
}
