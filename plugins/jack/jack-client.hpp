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

#ifndef __SWITCHER_JACK_CLIENT_H__
#define __SWITCHER_JACK_CLIENT_H__

#include <jack/jack.h>
#include <memory>
#include <atomic>
#include <functional>
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {

using jack_sample_t = jack_default_audio_sample_t;

class JackClient : public SafeBoolIdiom {
  // warning, number of missed samples is estimated from the xrun duration.
  using XRunCallback_t = std::function<void(uint number_of_missed_samples)>;
 public:
  explicit JackClient(const char *name);
  JackClient() = delete;
  JackClient(const JackClient &) = delete;
  JackClient &operator=(const JackClient &) = delete;
  jack_nframes_t get_sample_rate() const{return sample_rate_;};
  jack_nframes_t get_buffer_size() const{return buffer_size_;};
  void set_jack_process_callback(JackProcessCallback cb, void *arg);
  // the xrun callback is called in jack_process
  // before calling the actual process function:
  void set_on_xrun_callback(XRunCallback_t cb);
  jack_client_t *get_raw();
  
 private:
  using jack_client_handle =
      std::unique_ptr<jack_client_t, decltype(&jack_client_close)>;
  jack_client_handle client_;
  jack_status_t status_{};
  jack_nframes_t sample_rate_{0};  // actually uint32_t
  jack_nframes_t buffer_size_{0};  // actually uint32_t
  JackProcessCallback user_cb_{nullptr};
  void *user_cb_arg_{nullptr};
  std::atomic_uint xrun_count_{0};
  XRunCallback_t xrun_cb_{};
  bool safe_bool_idiom() const final;
  static void on_jack_shutdown (void *arg);
  static int jack_process (jack_nframes_t nframes, void *arg);
  static int on_xrun(void *arg);
};

class JackPort: public SafeBoolIdiom {
 public:
  JackPort(JackClient &client,
           unsigned int number,
           bool is_output = true);
  inline jack_port_t *get_raw(){return port_.get();}
 private:
    using port_handle =
        std::unique_ptr<jack_port_t, std::function<void(jack_port_t *)>>;
  port_handle port_;
  bool safe_bool_idiom() const final;
};

}  // namespace switcher
#endif
