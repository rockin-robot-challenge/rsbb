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

#include "benchmark_errors_and_warning_messages.h"

#include <QStringList>
#include <QMessageBox>
#include <QScrollBar>

#include <pluginlib/class_list_macros.h>

#include <roah_utils.h>

#include <ros_roah_rsbb.h>

using namespace std;
using namespace ros;

namespace rqt_roah_rsbb {
BenchmarkErrorsAndWarningMessages::BenchmarkErrorsAndWarningMessages() :
		rqt_gui_cpp::Plugin(), widget_(0), nh_(ros::NodeHandle()) {
	setObjectName("BenchmarkErrorsAndWarningMessages");
}

void BenchmarkErrorsAndWarningMessages::initPlugin(qt_gui_cpp::PluginContext& context) {
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QWidget();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);

	connect(ui_.clear_button, SIGNAL(released()), this, SLOT(clear()));

	// add widget to the user interface
	context.addWidget(widget_);

	ui_.table->setRowCount(1);
	ui_.table->setItem(0, 0, new QTableWidgetItem(""));
	ui_.table->setItem(0, 1, new QTableWidgetItem(""));
	ui_.table->setItem(0, 2, new QTableWidgetItem(QString::fromStdString("No messages to display")));

	rosout_subscriber_ = nh_.subscribe("/rosout", 1000, &BenchmarkErrorsAndWarningMessages::rosout_callback, this);

}

void BenchmarkErrorsAndWarningMessages::shutdownPlugin() {
	rosout_subscriber_.shutdown();
}

void BenchmarkErrorsAndWarningMessages::rosout_callback(const rosgraph_msgs::Log::ConstPtr& m) {
	Time now = Time::now();

	auto level = m->level;

	if (level & (rosgraph_msgs::Log::ERROR | rosgraph_msgs::Log::WARN | rosgraph_msgs::Log::FATAL)) {

		QString message_name = QString::fromStdString(m->name);
		QString message_string = QString::fromStdString(m->msg);

		QString message_string_with_repetitions;
		QString message_level_string;
		QColor message_level_color;

		switch (level) {
		case rosgraph_msgs::Log::WARN:
			message_level_string = QString("WARN ");
			message_level_color = Qt::yellow;
			break;
		case rosgraph_msgs::Log::ERROR:
			message_level_string = QString("ERROR");
			message_level_color = Qt::red;
			break;
		case rosgraph_msgs::Log::FATAL:
			message_level_string = QString("FATAL");
			message_level_color = Qt::red;
			break;
		}

		if(last_message_level_ == message_level_string && last_message_name_ == message_name && last_message_string_ == message_string){
			// note: this implies clear_ == false, but insertRow is not done to substitute the first row that is repeated
			last_message_repetitions_++;
			message_string_with_repetitions = QString("(x") + QString::fromStdString(to_string(last_message_repetitions_)) + QString(") ") + message_string;
		}else{
			last_message_repetitions_ = 1;
			message_string_with_repetitions = message_string;

			if(!clear_){
				// insert a new row if the message is not a repetition of the last and if the table is not clear
				ui_.table->insertRow(0);
			}
		}

		clear_ = false;

		last_message_level_ = message_level_string;
		last_message_name_ = message_name;
		last_message_string_ = message_string;

		QTableWidgetItem* level_item = new QTableWidgetItem(message_level_string);
		QTableWidgetItem* name_item  = new QTableWidgetItem(message_name);
		QTableWidgetItem* msg_item  = new QTableWidgetItem(message_string_with_repetitions);

		level_item->setBackgroundColor(message_level_color);

		ui_.table->setItem(0, 0, level_item);
		ui_.table->setItem(0, 1, name_item);
		ui_.table->setItem(0, 2, msg_item);

		ui_.table->verticalScrollBar()->setSliderPosition(ui_.table->verticalScrollBar()->minimum());

	}
}

void BenchmarkErrorsAndWarningMessages::clear() {

	ui_.table->setRowCount(1);
	ui_.table->setItem(0, 0, new QTableWidgetItem(""));
	ui_.table->setItem(0, 1, new QTableWidgetItem(""));
	ui_.table->setItem(0, 2, new QTableWidgetItem(QString::fromStdString("No messages to display (cleared)")));

	clear_ = true;

	last_message_repetitions_ = 1;

	last_message_level_ = QString();
	last_message_name_ = QString();
	last_message_string_ = QString();

}
}

PLUGINLIB_DECLARE_CLASS(rqt_roah_rsbb, BenchmarkErrorsAndWarningMessages, rqt_roah_rsbb::BenchmarkErrorsAndWarningMessages, rqt_gui_cpp::Plugin)
