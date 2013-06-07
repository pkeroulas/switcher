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


#ifndef __SWITCHER_HTTP_SDP_DEC_H__
#define __SWITCHER_HTTP_SDP_DEC_H__

#include "base-source.h"
#include "gst-element-cleaner.h"
#include "string-map.h"
#include <memory>

namespace switcher
{

  class HTTPSDPDec : public BaseSource, public GstElementCleaner
  {
  public:
    typedef std::shared_ptr<HTTPSDPDec> ptr;
    ~HTTPSDPDec();
    bool init ();
    bool to_shmdata (std::string uri);
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

  private: 
   GstElement *souphttpsrc_;
   GstElement *sdpdemux_;
   std::vector<GstElement *> decodebins_;
   StringMap<int> media_counters_;
   GstPad *main_pad_;
   GstCaps *rtpgstcaps_;
   bool discard_next_uncomplete_buffer_;
   std::string runtime_name_;
   void init_httpsdpdec ();
   void destroy_httpsdpdec ();

   static void decodebin_pad_added_cb (GstElement* object, GstPad *pad, gpointer user_data);
   static void httpsdpdec_pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data);
   static gboolean to_shmdata_wrapped (gpointer uri, gpointer user_data);
   static void no_more_pads_cb (GstElement* object, gpointer user_data);
   static void source_setup_cb (GstElement *httpsdpdec, GstElement *source, gpointer user_data);
   static gboolean event_probe_cb (GstPad *pad, GstEvent * event, gpointer data);
   static gboolean rewind (gpointer user_data);
   static void unknown_type_cb (GstElement *bin, GstPad *pad, GstCaps *caps, gpointer user_data);
   static int autoplug_select_cb (GstElement *bin, GstPad *pad, GstCaps *caps, GstElementFactory *factory, gpointer user_data);
   //filtering uncomplete custum buffers
   static gboolean gstrtpdepay_buffer_probe_cb (GstPad * pad, GstMiniObject * mini_obj, gpointer user_data);
   static gboolean gstrtpdepay_event_probe_cb (GstPad *pad, GstEvent * event, gpointer user_data);
   void pad_to_shmdata_writer (GstElement *bin, GstPad *pad);
  };

}  // end of namespace

#endif // ifndef