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

#ifndef __SWITCHER_XVIMAGESINK_H__
#define __SWITCHER_XVIMAGESINK_H__

#include <gst/gst.h>
#include <memory>
#include "./single-pad-gst-sink.hpp"
#include "./unique-gst-element.hpp"
#include "./g-source-wrapper.hpp"

namespace switcher {
class Xvimagesink: public SinglePadGstSink {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(Xvimagesink);
  Xvimagesink(const std::string &);
  ~Xvimagesink();
  Xvimagesink(const Xvimagesink &) = delete;
  Xvimagesink &operator=(const Xvimagesink &) = delete;

 private:
  UGstElem sink_bin_;
  UGstElem queue_;
  UGstElem ffmpegcolorspace_;
  UGstElem xvimagesink_;
  bool init_gpipe() final;
  bool can_sink_caps(std::string caps) final;
};
}  // namespace switcher
#endif
