/*
 * This file is part of switcher-myplugin.
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

#undef NDEBUG  // get assert in release mode

#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include "switcher/quiddity-basic-test.hpp"
#include "switcher/quiddity-manager.hpp"

int main() {
  switcher::QuiddityManager::ptr manager = switcher::QuiddityManager::make_manager("test_manager");

  gchar* usr_plugin_dir = g_strdup_printf("./");
  manager->scan_directory_for_plugins(usr_plugin_dir);
  g_free(usr_plugin_dir);

  assert(switcher::QuiddityBasicTest::test_full(manager, "posture_scan3dGPU"));
  return 0;
}
