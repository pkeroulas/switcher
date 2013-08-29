/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-myplugin.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef __SWITCHER_PORTMIDI_SINK_H__
#define __SWITCHER_PORTMIDI_SINK_H__

#include "switcher/quiddity.h"
#include "portmidi-devices.h"
#include "switcher/custom-property-helper.h"

#include <memory>

namespace switcher
{
  
  class PortMidiSink : public Quiddity, public PortMidi
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PortMidiSink);
    ~PortMidiSink ();
  
  };
  
  SWITCHER_DECLARE_PLUGIN(PortMidiSink);
  
}  // end of namespace

#endif // ifndef
