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

#ifndef __RQT_ROAH_RSBB_PUBLIC_DISPLAY_H__
#define __RQT_ROAH_RSBB_PUBLIC_DISPLAY_H__

#include <QMainWindow>
#include <QTimer>

#include <ros/ros.h>

#include <ui_public_display.h>
#include <roah_rsbb/UInt8.h>
#include <roah_rsbb/CoreToPublic.h>

class PublicDisplay: public QMainWindow {
Q_OBJECT

public:
	explicit PublicDisplay();
	~PublicDisplay();

private:
	ros::NodeHandle nh_;
	Ui::PublicDisplay ui_;
	QTimer update_timer_;
	ros::ServiceServer screen_srv_;
	ros::Subscriber core_to_public_sub_;

	bool set_screen(roah_rsbb::UInt8::Request& req, roah_rsbb::UInt8::Response& res);

	void core_to_public(roah_rsbb::CoreToPublic::ConstPtr const& msg);

private slots:
	void update();
};

#endif
