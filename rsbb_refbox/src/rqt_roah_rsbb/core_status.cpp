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

#include "core_status.h"

#include <QStringList>
#include <QMessageBox>

#include <pluginlib/class_list_macros.h>

#include <std_srvs/Empty.h>

using namespace std;
using namespace ros;

namespace rqt_roah_rsbb {
CoreStatus::CoreStatus() :
		rqt_gui_cpp::Plugin(), widget_(0) {
	setObjectName("CoreStatus");
}

void CoreStatus::initPlugin(qt_gui_cpp::PluginContext& context) {
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QWidget();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);

	connect(ui_.shutdown, SIGNAL(released()), this, SLOT(exit()));

	// add widget to the user interface
	context.addWidget(widget_);

	core_rcv_.start("/core/to_gui", getNodeHandle());

	connect(&update_timer_, SIGNAL(timeout()), this, SLOT(update()));
	update_timer_.start(200);
}

void CoreStatus::shutdownPlugin() {
	update_timer_.stop();
	core_rcv_.stop();
}

void CoreStatus::update() {
	Time now = Time::now();

	Time core_status_time;
	auto core_status = core_rcv_.last(core_status_time);

	if (!core_status) {
		ui_.status->setText("Waiting for core...");
		return;
	}

	if ((now - core_status_time) > Duration(1)) {
		ui_.status->setText("No communication!");
		ui_.addr->setText("--");
		ui_.port->setText("--");
		ui_.robots->setText("--");
		return;
	}

	ui_.status->setText(QString::fromStdString(core_status->status));
	ui_.addr->setText(QString::fromStdString(core_status->addr));
	ui_.port->setText(QString::fromStdString(core_status->port));
	ui_.robots->setText(QString::number(core_status->active_robots.size()));

	ui_.bm_server_status->setText(QString::fromStdString(core_status->bm_server_status));

	switch (core_status->bm_server_status_color) {
	case roah_rsbb::CoreToGui::RED:
		ui_.bm_server_status->setStyleSheet("background-color:red;");
		break;
	case roah_rsbb::CoreToGui::YELLOW:
		ui_.bm_server_status->setStyleSheet("background-color:yellow;");
		break;
	case roah_rsbb::CoreToGui::NEUTRAL:
		ui_.bm_server_status->setStyleSheet("");
		break;
	}

	ui_.record_server_status->setText(QString::fromStdString(core_status->record_server_status));

	switch (core_status->record_server_status_color) {
	case roah_rsbb::CoreToGui::RED:
		ui_.record_server_status->setStyleSheet("background-color:red;");
		break;
	case roah_rsbb::CoreToGui::YELLOW:
		ui_.record_server_status->setStyleSheet("background-color:yellow;");
		break;
	case roah_rsbb::CoreToGui::NEUTRAL:
		ui_.record_server_status->setStyleSheet("");
		break;
	}

}

void CoreStatus::exit() {
	if (service::waitForService("roah_rsbb_shutdown", 500)) {
		std_srvs::Empty s;
		if (!service::call("roah_rsbb_shutdown", s)) {
			NODELET_ERROR("Error calling roah_rsbb_shutdown service");
		}
	} else {
		NODELET_ERROR("Could not find roah_rsbb_shutdown service");
	}
}
}

PLUGINLIB_DECLARE_CLASS(rqt_roah_rsbb, CoreStatus, rqt_roah_rsbb::CoreStatus, rqt_gui_cpp::Plugin)
