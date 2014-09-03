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

#include "./unique-gst-element.hpp"

namespace switcher {

UniqueGstElement::UniqueGstElement(const gchar *
                                   class_name):element_
                                               (gst_element_factory_make(class_name, nullptr),
                                                &GstUtils::gst_element_deleter) {
}

void UniqueGstElement::g_invoke(std::function < void (gpointer) > command) {
  command(G_OBJECT(element_.get()));
  return;
}

void UniqueGstElement::invoke(std::function < void (GstElement *) > command) {
  command(element_.get());
  return;
}
}
