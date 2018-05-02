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

#include "public_display.h"

#include <QStringList>
#include <QDesktopWidget>
#include <QHeaderView>

#include <roah_utils.h>
#include <ros_roah_rsbb.h>

using namespace std;
using namespace ros;

PublicDisplay::PublicDisplay() :
		nh_(), screen_srv_(nh_.advertiseService("screen", &PublicDisplay::set_screen, this)), core_to_public_sub_(
				nh_.subscribe("/core/to_public", 1, &PublicDisplay::core_to_public, this)) {
	setObjectName("PublicDisplay");

	ui_.setupUi(this);

	ui_.schedule->horizontalHeader()->setResizeMode(QHeaderView::Fixed);

	connect(&update_timer_, SIGNAL(timeout()), this, SLOT(update()));
	update_timer_.start(500);

	int numScreens = QApplication::desktop()->numScreens();
	if (numScreens > 1) {
		cerr << "------------------------------------------------------------------------" << endl;
		for (int i = 0; i < numScreens; ++i) {
			cerr << "rosservice call " << screen_srv_.getService() << " " << i << endl;
		}
		cerr << "------------------------------------------------------------------------" << endl << flush;
	}
	ROS_ERROR_STREAM("You need to click the window, press 'alt-space' and select 'Always On Top' for the panel to disappear!!!");

	update();
}

PublicDisplay::~PublicDisplay() {
	update_timer_.stop();
	shutdown();
}

bool PublicDisplay::set_screen(roah_rsbb::UInt8::Request& req, roah_rsbb::UInt8::Response& res) {
	QRect screenres = QApplication::desktop()->screenGeometry(req.data);
	move(QPoint(screenres.x(), screenres.y()));
	resize(screenres.width(), screenres.height());
	showFullScreen();

	return true;
}

void PublicDisplay::core_to_public(roah_rsbb::CoreToPublic::ConstPtr const& msg) {
	ui_.clock->setText(QString::fromStdString(msg->clock));

	ui_.schedule->setRowCount(msg->schedule.size());

	for (size_t r = 0; r < msg->schedule.size(); ++r) {
		QTableWidgetItem* c0 = new QTableWidgetItem(QString::fromStdString(msg->schedule.at(r).time));
		QTableWidgetItem* c1 = new QTableWidgetItem(QString::fromStdString(msg->schedule.at(r).team));
		QTableWidgetItem* c2 = new QTableWidgetItem(QString::fromStdString(msg->schedule.at(r).benchmark));
		QTableWidgetItem* c3 = new QTableWidgetItem(QString::number(msg->schedule.at(r).round));
		QTableWidgetItem* c4 = new QTableWidgetItem(QString::number(msg->schedule.at(r).run));
		if (msg->schedule.at(r).running) {
			QColor selected(247, 204, 6);
			c0->setTextColor(selected);
			c1->setTextColor(selected);
			c2->setTextColor(selected);
			c3->setTextColor(selected);
			c4->setTextColor(selected);
		}
		c0->setTextAlignment(Qt::AlignCenter);
		c1->setTextAlignment(Qt::AlignCenter);
		c2->setTextAlignment(Qt::AlignCenter);
		c3->setTextAlignment(Qt::AlignCenter);
		c4->setTextAlignment(Qt::AlignCenter);
		ui_.schedule->setItem(r, 0, c0);
		ui_.schedule->setItem(r, 1, c1);
		ui_.schedule->setItem(r, 2, c2);
		ui_.schedule->setItem(r, 3, c3);
		ui_.schedule->setItem(r, 4, c4);
	}
}

void PublicDisplay::update() {
	spinOnce();
	if (!ok()) {
		QApplication::quit();
	}

	int width = ui_.schedule->horizontalHeader()->width();
	ui_.schedule->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
	int smallw = 0;
	for (int i = 0; i < 5; ++i) {
		smallw += ui_.schedule->columnWidth(i);
	}
	for (int i = 0; i < 5; ++i) {
		ui_.schedule->setColumnWidth(i, width * ui_.schedule->columnWidth(i) / smallw);
	}
}
