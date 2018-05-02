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

#include "tablet_status.h"

#include <QStringList>
#include <QMessageBox>

#include <pluginlib/class_list_macros.h>

#include <std_srvs/Empty.h>

#include <roah_utils.h>



using namespace std;
using namespace ros;



namespace rqt_roah_rsbb
{
  TabletStatus::TabletStatus()
    : rqt_gui_cpp::Plugin()
    , widget_ (0)
    , WARN_DURATION (1.0)
    , last_call_rcvd_ (TIME_MIN)
    , last_call_time_ (TIME_MIN)
    , last_pos_rcvd_ (TIME_MIN)
    , last_pos_time_ (TIME_MIN)
  {
    setObjectName ("TabletStatus");
  }

  void TabletStatus::initPlugin (qt_gui_cpp::PluginContext& context)
  {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi (widget_);

    // add widget to the user interface
    context.addWidget (widget_);

    core_rcv_.start ("/core/to_gui", getNodeHandle());

    connect (&update_timer_, SIGNAL (timeout()), this, SLOT (update()));
    update_timer_.start (100);
  }

  void TabletStatus::shutdownPlugin()
  {
    update_timer_.stop();
    core_rcv_.stop();
  }

  void TabletStatus::update()
  {
    Time now = Time::now();

    auto core = core_rcv_.last ();

    if (! core) {
      return;
    }

    auto beacon = (core->tablet_last_beacon - now).toSec();
    if ( (-3 < beacon) && (beacon < 0)) {
      ui_.beacon->setText ("OK");
    }
    else if (beacon < -30) {
      ui_.beacon->setText ("OFFLINE");
    }
    else {
      ui_.beacon->setText (QString::number (beacon, 'f', 1));
    }

    ui_.map->setText (core->tablet_display_map ? QString ("Map") : QString ("Call Button"));

    if (core->tablet_call_time <= TIME_MIN) {
      ui_.call_warn->setText ("");
      ui_.call_time->setText ("--");
    }
    else {
      if (last_call_rcvd_ != core->tablet_call_time) {
        last_call_time_ = Time::now();
        last_call_rcvd_ = core->tablet_call_time;
      }

      if ( (now - last_call_time_) < WARN_DURATION) {
        ui_.call_warn->setText ("!!!");
      }
      else {
        ui_.call_warn->setText ("");
      }
      ui_.call_time->setText (to_qstring (core->tablet_call_time));
    }

    if (core->tablet_position_time <= TIME_MIN) {
      ui_.pos_warn->setText ("");
      ui_.pos_time->setText ("--");
      ui_.x->setText ("--");
      ui_.y->setText ("--");
    }
    else {
      if (last_pos_rcvd_ != core->tablet_position_time) {
        last_pos_time_ = Time::now();
        last_pos_rcvd_ = core->tablet_position_time;
      }

      if ( (now - last_pos_time_) < WARN_DURATION) {
        ui_.pos_warn->setText ("!!!");
      }
      else {
        ui_.pos_warn->setText ("");
      }
      ui_.pos_time->setText (to_qstring (core->tablet_position_time));
      ui_.x->setText (QString::number (core->tablet_position_x, 'f', 3));
      ui_.y->setText (QString::number (core->tablet_position_y, 'f', 3));
    }
  }
}



PLUGINLIB_DECLARE_CLASS (rqt_roah_rsbb, TabletStatus, rqt_roah_rsbb::TabletStatus, rqt_gui_cpp::Plugin)
