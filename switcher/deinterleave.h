/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
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


#ifndef __SWITCHER_DEINTERLEAVE_H__
#define __SWITCHER_DEINTERLEAVE_H__

#include "switcher/base-sink.h"
#include <memory>

namespace switcher
{

  class Deinterleave : public BaseSink, public GstElementCleaner
  {
  public:
    typedef std::shared_ptr<Deinterleave> ptr;
    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

  private: 
   GstElement *deinterleave_;
   StringMap<int> media_counters_;
   static void make_deinterleave_active (ShmdataReader *caller, void *deinterleave_instance);
   static void pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data);
   static void no_more_pads_cb (GstElement* object, gpointer user_data);
  };

}  // end of namespace

#endif // ifndef