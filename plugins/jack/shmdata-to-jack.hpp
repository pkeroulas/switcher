/*
 * This file is part of switcher-jack.
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

#ifndef __SWITCHER_SHMDATA_TO_JACK_H__
#define __SWITCHER_SHMDATA_TO_JACK_H__

#include <memory>
#include <mutex>
#include "switcher/single-pad-gst-sink.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/custom-property-helper.hpp"
#include "switcher/unique-gst-element.hpp"
#include "./jack-client.hpp"

namespace switcher {
class ShmdataToJack: public SinglePadGstSink, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(ShmdataToJack);
  ShmdataToJack(const std::string &);
  ~ShmdataToJack() = default;
  ShmdataToJack(const ShmdataToJack &) = delete;
  ShmdataToJack &operator=(const ShmdataToJack &) = delete;

 private:
  JackClient jack_client_;
  GstElement *jacksink_{nullptr};
  GstElement *volume_{nullptr};
  GstElement *fakesink_{nullptr};
  gulong handoff_handler_{0};
  unsigned short channels_{0};
  CustomPropertyHelper::ptr custom_props_{};
  std::vector<jack_port_t *> output_ports_{};  // FIXME
  std::mutex output_ports_mutex_{};
  bool init_gpipe() final;
  bool start() final;
  bool stop() final;
  void on_shmdata_disconnect() final;
  void on_shmdata_connect(std::string shmdata_sochet_path) final;
  bool can_sink_caps(std::string caps) final;
  bool make_elements();
  void check_output_ports(int channels);
  static void on_handoff_cb(GstElement *object,
                            GstBuffer *buf,
                            GstPad *pad,
                            gpointer user_data);
  static int jack_process (jack_nframes_t nframes, void *arg);
};

SWITCHER_DECLARE_PLUGIN(ShmdataToJack);
}  // namespace switcher
#endif
