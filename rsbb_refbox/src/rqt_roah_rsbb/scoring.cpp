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

#include "scoring.h"

#include <QStringList>
#include <QMessageBox>
#include <QGroupBox>
#include <QLabel>
#include <QCheckBox>
#include <QSpinBox>

#include <pluginlib/class_list_macros.h>

#include <roah_utils.h>
#include <ros_roah_rsbb.h>

using namespace std;
using namespace ros;

namespace std {
inline bool operator==(const roah_rsbb::ZoneScoreGroup& lhs, const roah_rsbb::ZoneScoreGroup& rhs) {
	return lhs.group_name == rhs.group_name && lhs.types == rhs.types && lhs.current_values == rhs.current_values && lhs.descriptions == rhs.descriptions;
}
inline bool operator!=(const roah_rsbb::ZoneScoreGroup& lhs, const roah_rsbb::ZoneScoreGroup& rhs) {
	return !(lhs == rhs);
}
}

namespace rqt_roah_rsbb {
Scoring::Scoring() :
		rqt_gui_cpp::Plugin(), widget_(0), CONTROL_DURATION(1.0), last_control_(TIME_MIN) {
	setObjectName("Scoring");
}

void Scoring::initPlugin(qt_gui_cpp::PluginContext& context) {
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QScrollArea();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);

	// add widget to the user interface
	context.addWidget(widget_);

	core_rcv_.start("/core/to_gui", getNodeHandle());

	connect(&update_timer_, SIGNAL(timeout()), this, SLOT(update()));
	update_timer_.start(500);
}

void Scoring::shutdownPlugin() {
	update_timer_.stop();
	core_rcv_.stop();
}

void Scoring::update() {
	Time now = Time::now();

	auto core_status = core_rcv_.last();
	if (!core_status) {
		return;
	}

	if ((now - last_control_) < CONTROL_DURATION) {
		return;
	}

	string current_zone = param_direct("current_zone", string());
	for (roah_rsbb::ZoneState const& zone : core_status->zones) {
		if (zone.zone == current_zone) {
			if (zone.scoring == last_scoring_) {
				return;
			}

			QWidget().setLayout(ui_.layout);
			ui_.setupUi(widget_);
			service_template_.clear();

			last_scoring_ = zone.scoring;

			for (roah_rsbb::ZoneScoreGroup const& score_group : zone.scoring) {
				auto gridGroupBox = new QGroupBox(QString::fromStdString(score_group.group_name));
				QGridLayout* layout = new QGridLayout;

				for (size_t i = 0; i < score_group.types.size(); ++i) {
					switch (score_group.types[i]) {
					case roah_rsbb::ZoneScoreGroup::SCORING_BOOL: {
						QCheckBox* checkbox = new QCheckBox();
						checkbox->setCheckState(score_group.current_values.at(i) ? Qt::Checked : Qt::Unchecked);
						checkbox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
						QHBoxLayout* pLayout = new QHBoxLayout();
						pLayout->addWidget(checkbox);
						pLayout->setAlignment(Qt::AlignCenter);
						pLayout->setContentsMargins(0, 0, 0, 0);
						layout->addLayout(pLayout, i, 0);

						connect(checkbox, SIGNAL(stateChanged (int)), this, SLOT(check_cb (int)));

						roah_rsbb::ZoneScore tmp;
						tmp.request.zone = current_zone;
						tmp.request.score.group = score_group.group_name;
						tmp.request.score.desc = score_group.descriptions.at(i);
						service_template_[checkbox] = tmp;
					}
						break;
					case roah_rsbb::ZoneScoreGroup::SCORING_UINT: {
						auto spinbox = new QSpinBox();
						spinbox->setValue(score_group.current_values.at(i));
						layout->addWidget(spinbox, i, 0);

						connect(spinbox, SIGNAL(valueChanged (int)), this, SLOT(spin_cb (int)));

						roah_rsbb::ZoneScore tmp;
						tmp.request.zone = current_zone;
						tmp.request.score.group = score_group.group_name;
						tmp.request.score.desc = score_group.descriptions.at(i);
						service_template_[spinbox] = tmp;
					}
						break;
					}
					layout->addWidget(new QLabel(QString::fromStdString(score_group.descriptions.at(i))), i, 1);
				}

				layout->setColumnStretch(1, 1);
				gridGroupBox->setLayout(layout);
				ui_.layout->addWidget(gridGroupBox);
			}

			return;
		}
	}

	QWidget().setLayout(ui_.layout);
	ui_.setupUi(widget_);
}

void Scoring::check_cb(int value) {
	auto t_i = service_template_.find(sender());
	if (t_i != service_template_.end()) {
		roah_rsbb::ZoneScore tmp = t_i->second;
		tmp.request.score.value = (value == Qt::Unchecked) ? 0 : 1;
		ROS_INFO_STREAM("Setting \"" << tmp.request.score.desc << "\" to " << value);
		call_service("/core/set_score", tmp);
	} else {
		ROS_WARN("Did not find checkbox in service_template_, thus not calling service and not setting score.");
	}
	last_control_ = Time::now();
}

void Scoring::spin_cb(int value) {
	auto t_i = service_template_.find(sender());
	if (t_i != service_template_.end()) {
		roah_rsbb::ZoneScore tmp = t_i->second;
		tmp.request.score.value = value;
		ROS_INFO_STREAM("Setting \"" << tmp.request.score.desc << "\" to " << value);
		call_service("/core/set_score", tmp);
	} else {
		ROS_WARN("Did not find spinner in service_template_, thus not calling service and not setting score.");
	}
	last_control_ = Time::now();
}
}

PLUGINLIB_DECLARE_CLASS(rqt_roah_rsbb, Scoring, rqt_roah_rsbb::Scoring, rqt_gui_cpp::Plugin)
