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

#include "switcher/rtp-destination.h"

namespace switcher
{
  
  void 
  RtpDestination::set_host_name (std::string host_name)
  {
    host_name_ = host_name;
  }

  
  std::string
  RtpDestination::get_host_name ()
  {
    return host_name_;
  }

  bool 
  RtpDestination::add_stream (ShmdataReader::ptr rtp_shmdata_reader, std::string port)
  {
    ports_.insert (port,rtp_shmdata_reader);
    source_streams_.insert (rtp_shmdata_reader->get_path (),port);
    return true;
  }

  bool
  RtpDestination::remove_stream (std::string shmdata_stream_path)
  {
    if (!source_streams_.contains (shmdata_stream_path))
      {
	g_printerr ("RtpDestination: stream not found, cannot remove %s\n", 
		    shmdata_stream_path.c_str ());
	return false;
      }
    std::string port = source_streams_.lookup (shmdata_stream_path);
    source_streams_.remove (shmdata_stream_path);
    ports_.remove (port);
    return true;
  }
  
  std::string 
  RtpDestination::get_sdp ()
  {
    GstSDPMessage *sdp_description;
    //prepare SDP description
    gst_sdp_message_new (&sdp_description);
    
    /* some standard things first */
    gst_sdp_message_set_version (sdp_description, "0");
    
    //FIXME check and chose between IP4 and IP6, IP4 hardcoded
    gst_sdp_message_set_origin (sdp_description, 
      				"-",                // the user name
      				"1188340656180883", // a session id
      				"1",                // a session version
      				"IN",               // a network type
      				"IP4",              // an address type
      				"localhost"); //an address

    gst_sdp_message_set_session_name (sdp_description, "switcher session");
    gst_sdp_message_set_information (sdp_description, "telepresence");
    gst_sdp_message_add_time (sdp_description, "0", "0", NULL);
    gst_sdp_message_add_attribute (sdp_description, "tool", "switcher (based on GStreamer)");
    gst_sdp_message_add_attribute (sdp_description, "type", "broadcast");
    gst_sdp_message_add_attribute (sdp_description, "control", "*");

    //GstCaps *media_caps = gst_caps_from_string ("video/x-raw-yuv, format=(fourcc)YUY2, width=(int)320, height=(int)240, framerate=(fraction)30/1, color-matrix=(string)sdtv, chroma-site=(string)mpeg2");
    //GstCaps *media_caps = gst_caps_from_string ("video/x-h264, width=(int)320, height=(int)240, framerate=(fraction)30/1, pixel-aspect-ratio=(fraction)1/1, codec_data=(buffer)014d4015ffe10017674d4015eca0a0fd8088000003000bb9aca00078b16cb001000468ebecb2, stream-format=(string)avc, alignment=(string)au, level=(string)2.1, profile=(string)main");

    GstCaps *media_caps = gst_caps_from_string ("application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, ssrc=(uint)3978370044, payload=(int)96, clock-base=(uint)3851066683, seqnum-base=(uint)14240");

    // guint i, n_streams;
    // gchar *rangestr;
     
    // if (media->status != GST_RTSP_MEDIA_STATUS_PREPARED)
    //   goto not_prepared;
     
    // n_streams = gst_rtsp_media_n_streams (media);
     
    // rangestr = gst_rtsp_media_get_range_string (media, FALSE);
    // gst_sdp_message_add_attribute (sdp, "range", rangestr);
    // g_free (rangestr);
     
    // for (i = 0; i < n_streams; i++) {
    GstSDPMedia *smedia;
    GstStructure *s;
    const gchar *caps_str, *caps_enc, *caps_params;
    gchar *tmp;
    gint caps_pt, caps_rate;
    guint n_fields, j;
    gboolean first;
    GString *fmtp;
     

    s = gst_caps_get_structure (media_caps, 0);
      
    gst_sdp_media_new (&smedia);
      
    /* get media type and payload for the m= line */
    caps_str = gst_structure_get_string (s, "media");
    gst_sdp_media_set_media (smedia, caps_str);

    gst_structure_get_int (s, "payload", &caps_pt);
    tmp = g_strdup_printf ("%d", caps_pt);
    gst_sdp_media_add_format (smedia, tmp);
    g_free (tmp);

    gst_sdp_media_set_port_info (smedia, 9000, 1);
    gst_sdp_media_set_proto (smedia, "RTP/AVP");

    /* for the c= line */
    gst_sdp_media_add_connection (smedia, "IN", "udp",
				  "127.0.0.1", //server ip 
				  16, 0);

    /* get clock-rate, media type and params for the rtpmap attribute */
    gst_structure_get_int (s, "clock-rate", &caps_rate);
    caps_enc = gst_structure_get_string (s, "encoding-name");
    caps_params = gst_structure_get_string (s, "encoding-params");

    if (caps_enc) {
      if (caps_params)
	tmp = g_strdup_printf ("%d %s/%d/%s", caps_pt, caps_enc, caps_rate,
			       caps_params);
      else
	tmp = g_strdup_printf ("%d %s/%d", caps_pt, caps_enc, caps_rate);

      gst_sdp_media_add_attribute (smedia, "rtpmap", tmp);
      g_free (tmp);
    }

    /* the config uri */
    tmp = g_strdup_printf ("stream=%d", 0);
    gst_sdp_media_add_attribute (smedia, "control", tmp);
    g_free (tmp);

    /* collect all other properties and add them to fmtp */
    fmtp = g_string_new ("");
    g_string_append_printf (fmtp, "%d ", caps_pt);
    first = TRUE;
    n_fields = gst_structure_n_fields (s);
    for (j = 0; j < n_fields; j++) {
      const gchar *fname, *fval;

      fname = gst_structure_nth_field_name (s, j);

      /* filter out standard properties */
      if (g_strcmp0 (fname, "media") == 0)
	continue;
      if (g_strcmp0 (fname, "payload") == 0)
	continue;
      if (g_strcmp0 (fname, "clock-rate") == 0)
	continue;
      if (g_strcmp0 (fname, "encoding-name") == 0)
	continue;
      if (g_strcmp0 (fname, "encoding-params") == 0)
	continue;
      if (g_strcmp0 (fname, "ssrc") == 0)
	continue;
      if (g_strcmp0 (fname, "clock-base") == 0)
	continue;
      if (g_strcmp0 (fname, "seqnum-base") == 0)
	continue;

      if ((fval = gst_structure_get_string (s, fname))) {
	g_string_append_printf (fmtp, "%s%s=%s", first ? "" : ";", fname, fval);
	first = FALSE;
      }
    }
    if (!first) {
      tmp = g_string_free (fmtp, FALSE);
      gst_sdp_media_add_attribute (smedia, "fmtp", tmp);
      g_free (tmp);
    } else {
      g_string_free (fmtp, TRUE);
    }
    gst_sdp_message_add_media (sdp_description, smedia);
    gst_sdp_media_free (smedia);
     
    g_print ("--- SDP: \n%s\n",gst_sdp_message_as_text (sdp_description));
    gst_sdp_message_free (sdp_description);

    return gst_sdp_message_as_text (sdp_description);
  }

}
