/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "switcher/pulse-sink.h"

namespace switcher
{

  PulseSink::PulseSink (QuiddityLifeManager::ptr life_manager)
  {
    life_manager_ = life_manager;
    make_pulse_sink ();
  }

  
  PulseSink::PulseSink ()
  {
    make_pulse_sink ();
  }

  void 
  PulseSink::make_pulse_sink ()
  {
    pulse_sink_ = gst_parse_bin_from_description ("audioconvert ! pulsesink sync=false",
						  TRUE,
						  NULL);

    //gst_element_factory_make ("pulsesink",NULL);
    
    //set the name before registering properties
    set_name (gst_element_get_name (pulse_sink_));
    // g_object_set (G_OBJECT (pulse_sink_), "sync", FALSE, NULL);

    set_sink_element (pulse_sink_);
  }
  

  QuiddityDocumentation PulseSink::doc_ ("audio sink", "pulsesink",
					 "Output sound to pulse server");

  QuiddityDocumentation 
  PulseSink::get_documentation ()
  {
    return doc_;
  }
  
}