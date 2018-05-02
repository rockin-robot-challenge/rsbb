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

#include "active_robots.h"

#include <QStringList>
#include <QMessageBox>

#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace ros;

namespace rqt_roah_rsbb {
ActiveRobots::ActiveRobots() :
		rqt_gui_cpp::Plugin(), widget_(0) {
	setObjectName("ActiveRobots");
}

void ActiveRobots::initPlugin(qt_gui_cpp::PluginContext& context) {
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QWidget();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);

	ui_.table->horizontalHeader()->setResizeMode(QHeaderView::Stretch);

	// add widget to the user interface
	context.addWidget(widget_);

	core_rcv_.start("/core/to_gui", getNodeHandle());

	connect(&update_timer_, SIGNAL(timeout()), this, SLOT(update()));
	update_timer_.start(100);
}

void ActiveRobots::shutdownPlugin() {
	update_timer_.stop();
	core_rcv_.stop();
}

void ActiveRobots::update() {
	Time now = Time::now();

	auto core_status = core_rcv_.last();

	if (!core_status) {
		ui_.table->setRowCount(0);
		return;
	}

//	ROS_INFO("benchmarking_robots");
//	for(auto br : core_status->benchmarking_robots){
//		ROS_INFO_STREAM(br);
//	}


	QColor warning_color = Qt::yellow, team_connected_color = Qt::darkCyan, none_color = Qt::white;

	ui_.table->setRowCount(core_status->active_robots.size());

	for (size_t r = 0; r < core_status->active_robots.size(); ++r) {

		auto skew = core_status->active_robots.at(r).skew.toSec();
		bool skew_ok = (-0.1 < skew) && (skew < 0.1);

		auto beacon = (core_status->active_robots.at(r).beacon - now).toSec();
		bool beacon_ok = (-3 < beacon) && (beacon < 0);

		bool team_connected = std::find(core_status->benchmarking_robots.begin(), core_status->benchmarking_robots.end(), core_status->active_robots.at(r).team) != core_status->benchmarking_robots.end();

		QTableWidgetItem* team_item = new QTableWidgetItem(QString::fromStdString(core_status->active_robots.at(r).team));
		QTableWidgetItem* robot_item = new QTableWidgetItem(QString::fromStdString(core_status->active_robots.at(r).robot));
		QTableWidgetItem* skew_item;// = new QTableWidgetItem();
		QTableWidgetItem* beacon_item;// = new QTableWidgetItem();

		QColor team_item_color = none_color, robot_item_color = none_color, skew_item_color = none_color, beacon_item_color = none_color;

		if(team_connected){
			team_item_color = team_connected_color;
			robot_item_color = team_connected_color;
			skew_item_color = team_connected_color;
			beacon_item_color = team_connected_color;
		}

		if (skew_ok) {
			skew_item = new QTableWidgetItem("OK");
		} else {
			skew_item = new QTableWidgetItem(QString::number(skew, 'f', 1));
			skew_item_color = warning_color;
		}

		if (beacon_ok) {
			beacon_item = new QTableWidgetItem("OK");
		} else {
			beacon_item = new QTableWidgetItem(QString::number(beacon, 'f', 1));
			beacon_item_color = warning_color;
		}

		team_item->setBackgroundColor(team_item_color);
		robot_item->setBackgroundColor(robot_item_color);
		skew_item->setBackgroundColor(skew_item_color);
		beacon_item->setBackgroundColor(beacon_item_color);

		ui_.table->setItem(r, 0, team_item);
		ui_.table->setItem(r, 1, robot_item);
		ui_.table->setItem(r, 2, skew_item);
		ui_.table->setItem(r, 3, beacon_item);

	}
}
}

PLUGINLIB_DECLARE_CLASS(rqt_roah_rsbb, ActiveRobots, rqt_roah_rsbb::ActiveRobots, rqt_gui_cpp::Plugin)
