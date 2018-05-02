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

#ifndef __RQT_ROAH_RSBB_OMF_SWITCHES_H__
#define __RQT_ROAH_RSBB_OMF_SWITCHES_H__

#include <map>

#include <QTimer>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

#include <ui_omf_switches.h>
#include <roah_rsbb/CoreToGui.h>
#include "topic_receiver.h"



namespace rqt_roah_rsbb
{
  class OmfSwitches
    : public rqt_gui_cpp::Plugin
  {
      Q_OBJECT

    public:
      OmfSwitches();
      virtual void initPlugin (qt_gui_cpp::PluginContext& context);
      virtual void shutdownPlugin();

    private:
      Ui::OmfSwitches ui_;
      QWidget* widget_;
      QTimer update_timer_;
      TopicReceiver<roah_rsbb::CoreToGui> core_rcv_;
      const ros::Duration CONTROL_DURATION;
      ros::Time last_control_;
      std::map <int, QPushButton*> number_to_buttons_;

      void disable();

    private slots:
      void update();
      void complete();
      void damaged (int value);
      void a();
      void b();
      void c();
      void d();
      void e();
      void f();
      void g();
      void h();
      void i();
      void j();
  };
}

#endif
