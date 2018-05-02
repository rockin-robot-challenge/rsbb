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

#ifndef __RQT_ROAH_RSBB_ACTIVE_ROBOTS_H__
#define __RQT_ROAH_RSBB_ACTIVE_ROBOTS_H__

#include <QTimer>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

#include <ui_active_robots.h>
#include <roah_rsbb/CoreToGui.h>
#include "topic_receiver.h"

namespace rqt_roah_rsbb {
class ActiveRobots: public rqt_gui_cpp::Plugin {
Q_OBJECT

public:
	ActiveRobots();
	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();

private:
	Ui::ActiveRobots ui_;
	QWidget* widget_;
	QTimer update_timer_;
	TopicReceiver<roah_rsbb::CoreToGui> core_rcv_;

private slots:
	void update();
};
}

#endif
