/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#include <glib/gprintf.h>
#include <memory>
#include "./http-sdp-dec.hpp"
#include "./gst-utils.hpp"
#include "./scope-exit.hpp"
#include "./std2.hpp"
#include "./g-source-wrapper.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    HTTPSDPDec,
    "HTTP/SDP Player",
    "network",
    "decode an sdp-described stream deliver through http and make shmdatas",
    "LGPL",
    "httpsdpdec",
    "Nicolas Bouillot");

HTTPSDPDec::HTTPSDPDec(const std::string &):
    gst_pipeline_(std2::make_unique<GstPipeliner>()),
    souphttpsrc_("souphttpsrc"),
    sdpdemux_("sdpdemux") {
}

bool HTTPSDPDec::init() {
  if (!souphttpsrc_ || !sdpdemux_)
    return false;
  install_method("To Shmdata",
                 "to_shmdata",
                 "get streams from sdp description over http, "
                 "accept also base64 encoded SDP string",
                 "success or fail",
                 Method::make_arg_description("URL",
                                              "url",
                                              "URL to the sdp file, or a base64 encoded SDP description",
                                              nullptr),
                 (Method::method_ptr)to_shmdata_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr), this);
  return true;
}

void HTTPSDPDec::init_httpsdpdec() {
  if (!UGstElem::renew(souphttpsrc_))
    g_warning("error renewing souphttpsrc_"); 
  if (!UGstElem::renew(sdpdemux_))
    g_warning("error renewing sdpdemux_"); 
  g_signal_connect(GST_BIN(sdpdemux_.get_raw()),
                   "element-added",
                   (GCallback) HTTPSDPDec::on_new_element_in_sdpdemux,
                   nullptr);
  g_object_set(G_OBJECT(sdpdemux_.get_raw()),
               "latency", 0, nullptr);
  g_object_set(G_OBJECT(sdpdemux_.get_raw()),
               "async-handling", TRUE, nullptr);
  g_signal_connect(G_OBJECT(sdpdemux_.get_raw()),
                   "pad-added", (GCallback)httpsdpdec_pad_added_cb, this);
}

void HTTPSDPDec::destroy_httpsdpdec() {
  make_new_error_handler();
  gst_pipeline_ = std2::make_unique<GstPipeliner>();
  // FIXME:
  // clear_shmdatas();
  // reset_bin();
  // FIXME:
  // reset_counter_map();
}

void
HTTPSDPDec::on_new_element_in_sdpdemux(GstBin */*bin*/,
                                       GstElement */*element*/,
                                       gpointer /*user_data*/) {
  // FIXME add that in uridecodebin
  //g_object_set(G_OBJECT(element), "ntp-sync", TRUE, nullptr);
}

void HTTPSDPDec::make_new_error_handler() {
      on_error_.emplace_back(
          std2::make_unique<GSourceWrapper>([&](){uri_to_shmdata();},
                                            retry_delay_,
                                            true));
      // cleaning old sources
      if (2 < on_error_.size())
        on_error_.pop_front();
}

void HTTPSDPDec::httpsdpdec_pad_added_cb(GstElement */*object */,
                                         GstPad *pad,
                                         gpointer user_data) {
  HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
  GstPipeliner *gpipe = static_cast<GstPipeliner *>(user_data);
  std::unique_ptr<DecodebinToShmdata> decodebin = 
      std2::make_unique<DecodebinToShmdata>(gpipe);
  auto caps = gst_pad_get_current_caps(pad);
  On_scope_exit{gst_caps_unref(caps);};
  auto structure = gst_caps_get_structure(caps, 0);
  decodebin->set_media_label(gst_structure_get_string (structure, "media-label"));
  decodebin->invoke_with_return<gboolean>(
      std::bind(gst_bin_add, GST_BIN(context->gst_pipeline_->get_pipeline()),
                std::placeholders::_1));
  // GstPad *sinkpad = gst_element_get_static_pad (decodebin, "sink");
  auto get_pad = std::bind(gst_element_get_static_pad,
                           std::placeholders::_1,
                           "sink");
  GstPad *sinkpad =
      decodebin->invoke_with_return<GstPad *>(std::move(get_pad));
  On_scope_exit {gst_object_unref(GST_OBJECT(sinkpad));};
  GstUtils::check_pad_link_return(gst_pad_link(pad, sinkpad));
  decodebin->invoke(std::bind(GstUtils::sync_state_with_parent,
                              std::placeholders::_1));
  context->decodebins_.push_back(std::move(decodebin));
}

gboolean HTTPSDPDec::to_shmdata_wrapped(gpointer uri, gpointer user_data) {
  HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
  if (context->to_shmdata((char *)uri))
    return TRUE;
  else
    return FALSE;
}

bool HTTPSDPDec::to_shmdata(std::string uri) {
  if (uri.find("http://") == 0) {
    souphttpsrc_.mute("souphttpsrc");
    uri_ = std::move(uri);
  } else {
    souphttpsrc_.mute("dataurisrc");
    uri_ =  std::string("data:application/sdp;base64," + uri);
  }
  on_error_.clear();
  uri_to_shmdata();
  return true;
}

void HTTPSDPDec::uri_to_shmdata() {
  destroy_httpsdpdec();
  //reset_bin();
  init_httpsdpdec();
  g_object_set_data(G_OBJECT(sdpdemux_.get_raw()),
                    "on-error-gsource",
                    (gpointer)on_error_.back().get());
  g_debug("httpsdpdec: to_shmdata set uri %s", uri_.c_str());
  // for souphttpsrc
  g_object_set(G_OBJECT(souphttpsrc_.get_raw()),
               "location", uri_.c_str(), nullptr);
  // for dataurisrc
  g_object_set(G_OBJECT(souphttpsrc_.get_raw()),
               "uri", uri_.c_str(), nullptr);
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   souphttpsrc_.get_raw(),
                   sdpdemux_.get_raw(),
                   nullptr);
  gst_element_link(souphttpsrc_.get_raw(), sdpdemux_.get_raw());
  gst_pipeline_->play(true);
}

}  // namespace switcher
