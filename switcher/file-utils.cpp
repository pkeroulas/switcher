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

#include "./file-utils.hpp"
#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include "./scope-exit.hpp"
#include "./string-utils.hpp"

namespace switcher {

std::pair<bool, std::string> FileUtils::prepare_writable_dir(const std::string& path) {
  if (is_dir(path)) {
    if (0 != access(path.c_str(), W_OK | X_OK)) {  // not writable
      int err = errno;
      return std::make_pair(false, std::string(strerror(err)));
    }
    return std::make_pair(true, std::string());
  }

  // trying to create the directory
  std::size_t i = 0;
  while (std::string::npos != i) {
    std::size_t found = path.find('/', i);
    if (i != found) {
      auto dir = std::string(path, 0, found);
      if (!is_dir(dir)) {
        auto res = create_writable_dir(dir);
        if (!res.first) return res;
      }
    }
    if (std::string::npos != found) {
      i = found + 1;
    } else {
      i = std::string::npos;
    }
  }
  return std::make_pair(true, std::string());
}

bool FileUtils::is_dir(const std::string& path) {
  struct stat sb;
  if (stat(path.c_str(), &sb) == -1)  // does not exists
    return false;
  if ((sb.st_mode & S_IFMT) != S_IFDIR)  // is not a directory
    return false;
  return true;
}

std::pair<bool, std::string> FileUtils::create_writable_dir(const std::string& path) {
  if (-1 == mkdir(path.c_str(), S_IRWXU)) {
    int err = errno;
    return std::make_pair(false, std::string(strerror(err)));
  }
  return std::make_pair(true, std::string());
}

std::vector<std::string> FileUtils::get_files_from_directory(std::string path,
                                                             std::string prefix,
                                                             std::string suffix,
                                                             bool recursive) {
  DIR* directory;
  struct dirent* entry;
  std::vector<std::string> files;

  directory = opendir(path.c_str());
  if (!directory) return files;
  On_scope_exit { closedir(directory); };

  while ((entry = readdir(directory))) {
    if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..") continue;

    if (entry->d_type == DT_REG) {
      if ((prefix.empty() || StringUtils::starts_with(entry->d_name, prefix)) &&
          (suffix.empty() || StringUtils::ends_with(entry->d_name, suffix))) {
        files.push_back(path + "/" + entry->d_name);
      }
    } else if (entry->d_type == DT_DIR && recursive) {
      auto inner_folder_files =
          get_files_from_directory(path + "/" + entry->d_name, prefix, suffix, recursive);
      files.insert(files.end(), inner_folder_files.begin(), inner_folder_files.end());
    }
  }

  return files;
}

}  // namespace switcher
