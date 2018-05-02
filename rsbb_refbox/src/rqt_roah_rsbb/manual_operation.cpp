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

#include "manual_operation.h"

#include <QStringList>
#include <QMessageBox>

#include <pluginlib/class_list_macros.h>

#include <roah_utils.h>

#include <ros_roah_rsbb.h>

#include <roah_rsbb/Zone.h>
#include <roah_rsbb/ZoneManualOperationResult.h>

#include <std_srvs/Empty.h>

using namespace std;
using namespace ros;

namespace rqt_roah_rsbb {
ManualOperation::ManualOperation() :
		rqt_gui_cpp::Plugin(), widget_(0) {
	setObjectName("ManualOperation");
}

void ManualOperation::initPlugin(qt_gui_cpp::PluginContext& context) {
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QWidget();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);

	connect(ui_.mo_complete, SIGNAL(released()), this, SLOT(complete()));

	// add widget to the user interface
	context.addWidget(widget_);

	core_rcv_.start("/core/to_gui", getNodeHandle());

	connect(&update_timer_, SIGNAL(timeout()), this, SLOT(update()));
	update_timer_.start(200);

	default_palette_ = ui_.mo->palette();
	warn_palette_ = ui_.mo->palette();
	warn_palette_.setColor(QPalette::All, QPalette::Base, Qt::yellow);
	warn_palette_.setColor(QPalette::All, QPalette::Text, Qt::black);
}

void ManualOperation::shutdownPlugin() {
	update_timer_.stop();
	core_rcv_.stop();
}

void ManualOperation::update() {
	auto core = core_rcv_.last();
	string current_zone = param_direct("current_zone", string());

	if (core) {
		for (roah_rsbb::ZoneState const& zone : core->zones) {
			if ((zone.zone == current_zone) && (!zone.manual_operation.empty())) {
				ui_.mo->setPalette(warn_palette_);
				ui_.mo_complete->setEnabled(true);
				ui_.mo->setPlainText(QString::fromStdString(zone.manual_operation));
				return;
			}
		}
	}

	ui_.mo->setPalette(default_palette_);
	ui_.mo_complete->setEnabled(false);
	ui_.mo->setPlainText("");
}

void ManualOperation::complete() {
	roah_rsbb::ZoneManualOperationResult z;
	z.request.zone = param_direct("current_zone", string());
	z.request.manual_operation_result = ui_.mo_result->toPlainText().toStdString();

	call_service("/core/manual_operation_complete", z);

	if (z.response.success){
		ui_.mo_result->setPlainText("");
	}
}
}

PLUGINLIB_DECLARE_CLASS(rqt_roah_rsbb, ManualOperation, rqt_roah_rsbb::ManualOperation, rqt_gui_cpp::Plugin)
