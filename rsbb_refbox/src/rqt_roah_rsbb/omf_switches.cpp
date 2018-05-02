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

#include "omf_switches.h"

#include <QStringList>
#include <QMessageBox>

#include <pluginlib/class_list_macros.h>

#include <roah_utils.h>

#include <ros_roah_rsbb.h>

#include <roah_rsbb/Zone.h>
#include <roah_rsbb/ZoneUInt8.h>

#include <std_srvs/Empty.h>



using namespace std;
using namespace ros;



const string STYLE_SHEET =
  "QPushButton               { border: none; border-radius: 8px; background-color: black;  color: white } "
  "QPushButton:checked       { border: none; border-radius: 8px; background-color: yellow; color: black } "
  "QPushButton:disabled      { border: none; border-radius: 8px; background-color: gray;   color: black } ";
/*
   QPushButton:focus         { border: 1px; border-radius: 8px; border-style: solid; border-color: gray; background-color: black;  color: white }
   QPushButton:focus:checked { border: 1px; border-radius: 8px; border-style: solid; border-color: gray; background-color: yellow; color: black }
*/



const int buttonsmap[] = {1, 10, 5, 6, 3, 2, 7, 8, 9, 4};



namespace rqt_roah_rsbb
{
  OmfSwitches::OmfSwitches()
    : rqt_gui_cpp::Plugin()
    , widget_ (0)
    , CONTROL_DURATION (0.5)
    , last_control_ (TIME_MIN)
  {
    setObjectName ("OmfSwitches");
  }

  void OmfSwitches::initPlugin (qt_gui_cpp::PluginContext& context)
  {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi (widget_);

    QString qstylesheet = QString::fromStdString (STYLE_SHEET);
    ui_.a->setStyleSheet (qstylesheet);
    ui_.b->setStyleSheet (qstylesheet);
    ui_.c->setStyleSheet (qstylesheet);
    ui_.d->setStyleSheet (qstylesheet);
    ui_.e->setStyleSheet (qstylesheet);
    ui_.f->setStyleSheet (qstylesheet);
    ui_.g->setStyleSheet (qstylesheet);
    ui_.h->setStyleSheet (qstylesheet);
    ui_.i->setStyleSheet (qstylesheet);
    ui_.j->setStyleSheet (qstylesheet);
    ui_.a->setText (QString::number (buttonsmap[0]));
    ui_.b->setText (QString::number (buttonsmap[1]));
    ui_.c->setText (QString::number (buttonsmap[2]));
    ui_.d->setText (QString::number (buttonsmap[3]));
    ui_.e->setText (QString::number (buttonsmap[4]));
    ui_.f->setText (QString::number (buttonsmap[5]));
    ui_.g->setText (QString::number (buttonsmap[6]));
    ui_.h->setText (QString::number (buttonsmap[7]));
    ui_.i->setText (QString::number (buttonsmap[8]));
    ui_.j->setText (QString::number (buttonsmap[9]));

    connect (ui_.a, SIGNAL (pressed()), this, SLOT (a()));
    connect (ui_.b, SIGNAL (pressed()), this, SLOT (b()));
    connect (ui_.c, SIGNAL (pressed()), this, SLOT (c()));
    connect (ui_.d, SIGNAL (pressed()), this, SLOT (d()));
    connect (ui_.e, SIGNAL (pressed()), this, SLOT (e()));
    connect (ui_.f, SIGNAL (pressed()), this, SLOT (f()));
    connect (ui_.g, SIGNAL (pressed()), this, SLOT (g()));
    connect (ui_.h, SIGNAL (pressed()), this, SLOT (h()));
    connect (ui_.i, SIGNAL (pressed()), this, SLOT (i()));
    connect (ui_.j, SIGNAL (pressed()), this, SLOT (j()));
    connect (ui_.damaged, SIGNAL (valueChanged (int)), this, SLOT (damaged (int)));
    connect (ui_.complete, SIGNAL (released()), this, SLOT (complete()));

    // add widget to the user interface
    context.addWidget (widget_);

    core_rcv_.start ("/core/to_gui", getNodeHandle());

    connect (&update_timer_, SIGNAL (timeout()), this, SLOT (update()));
    update_timer_.start (200);

    number_to_buttons_[buttonsmap[0]] = ui_.a;
    number_to_buttons_[buttonsmap[1]] = ui_.b;
    number_to_buttons_[buttonsmap[2]] = ui_.c;
    number_to_buttons_[buttonsmap[3]] = ui_.d;
    number_to_buttons_[buttonsmap[4]] = ui_.e;
    number_to_buttons_[buttonsmap[5]] = ui_.f;
    number_to_buttons_[buttonsmap[6]] = ui_.g;
    number_to_buttons_[buttonsmap[7]] = ui_.h;
    number_to_buttons_[buttonsmap[8]] = ui_.i;
    number_to_buttons_[buttonsmap[9]] = ui_.j;
  }

  void OmfSwitches::shutdownPlugin()
  {
    update_timer_.stop();
    core_rcv_.stop();
  }

  void OmfSwitches::disable()
  {
    ui_.a->setChecked (false);
    ui_.b->setChecked (false);
    ui_.c->setChecked (false);
    ui_.d->setChecked (false);
    ui_.e->setChecked (false);
    ui_.f->setChecked (false);
    ui_.g->setChecked (false);
    ui_.h->setChecked (false);
    ui_.i->setChecked (false);
    ui_.j->setChecked (false);

    ui_.a->setEnabled (false);
    ui_.b->setEnabled (false);
    ui_.c->setEnabled (false);
    ui_.d->setEnabled (false);
    ui_.e->setEnabled (false);
    ui_.f->setEnabled (false);
    ui_.g->setEnabled (false);
    ui_.h->setEnabled (false);
    ui_.i->setEnabled (false);
    ui_.j->setEnabled (false);
    ui_.damaged->setValue (0);
    ui_.damaged->setEnabled (false);
    ui_.complete->setEnabled (false);
  }

  void OmfSwitches::update()
  {
    Time now = Time::now();

    auto core = core_rcv_.last ();
    string current_zone = param_direct ("current_zone", string());

    if (core) {
      for (roah_rsbb::ZoneState const& zone : core->zones) {
        if (zone.zone == current_zone) {
          if (! zone.omf) {
            break;
          }

          if (! ui_.a->isEnabled()) {
            ui_.a->setEnabled (true);
            ui_.b->setEnabled (true);
            ui_.c->setEnabled (true);
            ui_.d->setEnabled (true);
            ui_.e->setEnabled (true);
            ui_.f->setEnabled (true);
            ui_.g->setEnabled (true);
            ui_.h->setEnabled (true);
            ui_.i->setEnabled (true);
            ui_.j->setEnabled (true);
            ui_.damaged->setEnabled (true);
          }

          ui_.complete->setEnabled (zone.omf_complete);

          if ( (now - last_control_) < CONTROL_DURATION) {
            return;
          }

          ui_.a->setChecked (false);
          ui_.b->setChecked (false);
          ui_.c->setChecked (false);
          ui_.d->setChecked (false);
          ui_.e->setChecked (false);
          ui_.f->setChecked (false);
          ui_.g->setChecked (false);
          ui_.h->setChecked (false);
          ui_.i->setChecked (false);
          ui_.j->setChecked (false);
          for (auto const& i : zone.omf_switches) {
            number_to_buttons_.at (i)->setChecked (true);
          }

          if (ui_.damaged->value() != static_cast<int> (zone.omf_damaged)) {
            ui_.damaged->setValue (zone.omf_damaged);
          }

          return;
        }
      }
    }

    disable();
  }

  void OmfSwitches::complete()
  {
    roah_rsbb::Zone z;
    z.request.zone = param_direct ("current_zone", string());
    call_service ("/core/omf_switches/complete", z);

    disable();
  }

  void OmfSwitches::damaged (int value)
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = value;
    call_service ("/core/omf_switches/damaged", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::a()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[0];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::b()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[1];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::c()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[2];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::d()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[3];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::e()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[4];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::f()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[5];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::g()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[6];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::h()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[7];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::i()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[8];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }

  void OmfSwitches::j()
  {
    roah_rsbb::ZoneUInt8 z;
    z.request.zone = param_direct ("current_zone", string());
    z.request.data = buttonsmap[9];
    call_service ("/core/omf_switches/button", z);
    last_control_ = Time::now();
  }
}



PLUGINLIB_DECLARE_CLASS (rqt_roah_rsbb, OmfSwitches, rqt_roah_rsbb::OmfSwitches, rqt_gui_cpp::Plugin)
