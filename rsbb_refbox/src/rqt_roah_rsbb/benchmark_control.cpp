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

#include <iostream>

#include "benchmark_control.h"

#include <QStringList>
#include <QMessageBox>
#include <QScrollBar>

#include <pluginlib/class_list_macros.h>

#include <roah_utils.h>

#include <ros_roah_rsbb.h>

#include <roah_rsbb/Zone.h>
#include <roah_rsbb/ZoneUInt8.h>

using namespace std;
using namespace ros;

namespace rqt_roah_rsbb {
BenchmarkControl::BenchmarkControl() :
		rqt_gui_cpp::Plugin(), widget_(0) {
	setObjectName("BenchmarkControl");
}

void BenchmarkControl::initPlugin(qt_gui_cpp::PluginContext& context) {
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QWidget();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);
	// add widget to the user interface
	context.addWidget(widget_);

	connect(ui_.zone, SIGNAL(currentIndexChanged (QString const&)), this, SLOT(zone (QString const&)));
	connect(ui_.run_selector, SIGNAL(valueChanged (int const&)), this, SLOT(run (int const&)));
	connect(ui_.connect, SIGNAL(clicked()), this, SLOT(connect_s()));
	connect(ui_.disconnect, SIGNAL(clicked()), this, SLOT(disconnect()));
	connect(ui_.start, SIGNAL(clicked()), this, SLOT(start()));
	connect(ui_.stop, SIGNAL(clicked()), this, SLOT(stop()));
	connect(ui_.previous, SIGNAL(clicked()), this, SLOT(previous()));
	connect(ui_.next, SIGNAL(clicked()), this, SLOT(next()));
	core_rcv_.start("/core/to_gui", getNodeHandle());

	connect(&update_timer_, SIGNAL(timeout()), this, SLOT(update()));
	update_timer_.start(200);
}

void BenchmarkControl::shutdownPlugin() {
	update_timer_.stop();
	core_rcv_.stop();
}

void BenchmarkControl::update() {
	Time now = Time::now();

	Time core_status_time;
	auto core_status = core_rcv_.last(core_status_time);

	if (!core_status) {
		return;
	}

	ui_.clock->setText(to_qstring(Time(core_status->clock.sec, 0)));

	param_direct("current_zone", string(), current_zone_);

	bool zone_changed = false;
	if (previous_zone_ != current_zone_){
		zone_changed = true;
		previous_zone_ = current_zone_;
	}

	roah_rsbb::ZoneState const* current_zone = nullptr;

	set<string> new_zones;
	for (roah_rsbb::ZoneState const& zone : core_status->zones) {
		new_zones.insert(zone.zone);
		if (zone.zone == current_zone_) {
			current_zone = &zone;
		}
	}

	if (known_zones_ != new_zones) {
		ui_.zone->clear();
		for (string const& zone : new_zones) {
			ui_.zone->addItem(QString::fromStdString(zone));
		}
		known_zones_ = move(new_zones);
	}

	if (current_zone) {
		QString current_index_text = ui_.zone->itemText(ui_.zone->currentIndex());
		QString new_zone = QString::fromStdString(current_zone_);

		if (current_index_text != new_zone) {
			ui_.zone->setCurrentIndex(ui_.zone->findText(new_zone)); // Guaranteed to exist by for loop above
		}

		bool benchmark_changed = false;
		if (previous_benchmark_ != current_zone->code || previous_schedule_ != to_qstring(current_zone->schedule)){
			benchmark_changed = true;
			previous_benchmark_ = current_zone->code;
			previous_schedule_ = to_qstring(current_zone->schedule);
		}

		if (zone_changed || benchmark_changed) {
			ui_.run_selector->setValue((int) current_zone->run);
		}

		ui_.desc->setText(QString::fromStdString(current_zone->desc + " (" + current_zone->code + ")"));
		ui_.timeout->setText(to_qstring(current_zone->timeout));
		ui_.team->setText(QString::fromStdString(current_zone->team));
		ui_.round->setText(QString::number(current_zone->run));
		ui_.run_selector->setEnabled(current_zone->run_selector_enabled);

//		ui_.name->setText(QString::fromStdString(current_zone->name));
//		ui_.code->setText(QString::fromStdString(current_zone->code));
//		ui_.run->setText(QString::number(current_zone->run));
//		ui_.sched->setText(to_qstring(current_zone->schedule));

		ui_.timer->setText(to_qstring(current_zone->timer));
		QString new_state = QString::fromStdString(current_zone->state);
		if (ui_.state->toPlainText() != new_state) {
			ui_.state->setPlainText(new_state);
			QScrollBar* sb = ui_.state->verticalScrollBar();
			sb->setValue(sb->maximum());
		}

		ui_.benchmark_state->setText(QString::fromStdString(current_zone->benchmark_state.empty() ? "--" : current_zone->benchmark_state));
		ui_.goal_state->setText(QString::fromStdString(current_zone->goal_state.empty() ? "--" :  current_zone->goal_state));
		ui_.manual_operation_state->setText(QString::fromStdString(current_zone->manual_operation_state.empty() ? "--" :  current_zone->manual_operation_state));
		ui_.bmscript_state->setText(QString::fromStdString(current_zone->bmscript_state.empty() ? "--" :  current_zone->bmscript_state));
		ui_.robot_state->setText(QString::fromStdString(current_zone->robot_state.empty() ? "--" :  current_zone->robot_state));

		ui_.connect->setEnabled(current_zone->connect_enabled);
		ui_.disconnect->setEnabled(current_zone->disconnect_enabled);
		ui_.start->setEnabled(current_zone->start_enabled);
		ui_.stop->setEnabled(current_zone->stop_enabled);
		ui_.previous->setEnabled(current_zone->prev_enabled);
		ui_.next->setEnabled(current_zone->next_enabled);

	} else {
		ui_.desc->setText("--");
		ui_.timeout->setText("--");
		ui_.team->setText("--");
		ui_.round->setText("--");

//		ui_.name->setText("--");
//		ui_.code->setText("--");
//		ui_.run->setText("--");
//		ui_.sched->setText("--");

		ui_.timer->setText("00:00");
		ui_.state->setPlainText("--");

		ui_.connect->setEnabled(false);
		ui_.disconnect->setEnabled(false);
		ui_.start->setEnabled(false);
		ui_.stop->setEnabled(false);
		ui_.previous->setEnabled(false);
		ui_.next->setEnabled(false);
	}
}

void BenchmarkControl::zone(QString const& zone) {
	NODELET_DEBUG_STREAM("Setting zone to: " << zone.toStdString());
	ros::param::set("current_zone", zone.toStdString());
}

void BenchmarkControl::run(int const& run) {
	roah_rsbb::ZoneUInt8 z;
	z.request.zone = current_zone_;
	z.request.data = run;
	NODELET_DEBUG_STREAM("Setting run to: " << run);
	call_service("/core/run_select", z);
}

void BenchmarkControl::connect_s() {
	roah_rsbb::Zone z;
	z.request.zone = current_zone_;
	call_service("/core/connect", z);
}

void BenchmarkControl::disconnect() {
	roah_rsbb::Zone z;
	z.request.zone = current_zone_;
	call_service("/core/disconnect", z);
}

void BenchmarkControl::start() {
	roah_rsbb::Zone z;
	z.request.zone = current_zone_;
	call_service("/core/start", z);
}

void BenchmarkControl::stop() {
	roah_rsbb::Zone z;
	z.request.zone = current_zone_;
	call_service("/core/stop", z);
}

void BenchmarkControl::previous() {
	roah_rsbb::Zone z;
	z.request.zone = current_zone_;
	call_service("/core/previous", z);
}

void BenchmarkControl::next() {
	roah_rsbb::Zone z;
	z.request.zone = current_zone_;
	call_service("/core/next", z);
}
}

PLUGINLIB_DECLARE_CLASS(rqt_roah_rsbb, BenchmarkControl, rqt_roah_rsbb::BenchmarkControl, rqt_gui_cpp::Plugin)
