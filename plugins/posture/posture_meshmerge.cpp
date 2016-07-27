/*
 *
 * posture is free software; you can redistribute it and/or
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

#include "./posture_meshmerge.hpp"

#include <iostream>

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureMeshMerge,
                                     "meshmerge",
                                     "Mesh Merge",
                                     "video",
                                     "reader/writer",
                                     "Merges meshes captured with 3D cameras",
                                     "LGPL",
                                     "Emmanuel Durand");

PostureMeshMerge::PostureMeshMerge(const std::string&) : shmcntr_(static_cast<Quiddity*>(this)) {}

PostureMeshMerge::~PostureMeshMerge() { stop(); }

bool PostureMeshMerge::start() {
  cameras_updated_.resize(source_id_);

  calibration_reader_ = unique_ptr<CalibrationReader>(new CalibrationReader(calibration_path_));
  merger_ = make_shared<MeshMerger>(source_id_);
  merger_->setCalibration(calibration_reader_->getCalibrationParams());
  merger_->setApplyCalibration(apply_calibration_);

  merger_->start();
  update_loop_started_ = true;
  update_thread_ = thread([&]() { update_loop(); });

  return true;
}

bool PostureMeshMerge::stop() {
  update_loop_started_ = false;
  unique_lock<mutex> lockUpdate(update_mutex_);
  update_cv_.notify_one();
  if (update_thread_.joinable()) update_thread_.join();

  lock_guard<mutex> lock(mutex_);
  if (merger_ != nullptr) {
    merger_->stop();
    merger_.reset();
  }

  return true;
}

bool PostureMeshMerge::init() {
  init_startable(this);

  shmcntr_.install_connect_method([this](const std::string path) { return connect(path); },
                                  [this](const std::string path) { return disconnect(path); },
                                  [this]() { return disconnect_all(); },
                                  [this](const std::string caps) { return can_sink_caps(caps); },
                                  8);

  pmanage<MPtr(&PContainer::make_string)>(
      "calibration_path",
      [this](const std::string& val) {
        calibration_path_ = val;
        if (calibration_reader_) {
          calibration_reader_->loadCalibration(calibration_path_);
          merger_->setCalibration(calibration_reader_->getCalibrationParams());
        }
        return true;
      },
      [this]() { return calibration_path_; },
      "Calibration path",
      "Path to the calibration file",
      calibration_path_);

  pmanage<MPtr(&PContainer::make_bool)>("reload_calibration",
                                        [this](const bool& val) {
                                          reload_calibration_ = val;
                                          return true;
                                        },
                                        [this]() { return reload_calibration_; },
                                        "Per frame calibration",
                                        "Reload calibration at each frame",
                                        reload_calibration_);

  pmanage<MPtr(&PContainer::make_bool)>("apply_calibration",
                                        [this](const bool& val) {
                                          apply_calibration_ = val;
                                          return true;
                                        },
                                        [this]() { return apply_calibration_; },
                                        "Apply calibration",
                                        "Apply loaded calibration to meshes",
                                        apply_calibration_);

  return true;
}

void PostureMeshMerge::update_loop() {
  while (update_loop_started_) {
    unique_lock<mutex> lock(update_mutex_);
    if (!update_wanted_) update_cv_.wait(lock);

    update_wanted_ = false;

    if (!update_loop_started_) break;

    if (reload_calibration_) {
      calibration_reader_->loadCalibration(calibration_path_);
      merger_->setCalibration(calibration_reader_->getCalibrationParams());
    }

    auto mesh = vector<unsigned char>();
    merger_->getMesh(mesh);

    if (mesh.size() != 0) {
      if (mesh_writer_ == nullptr ||
          mesh.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
        auto data_type = string(POLYGONMESH_TYPE_BASE);
        mesh_writer_.reset();
        mesh_writer_ = std::make_unique<ShmdataWriter>(
            this, make_file_name("mesh"), std::max(mesh.size() * 2, (size_t)1024), data_type);
      }

      mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
          const_cast<unsigned char*>(mesh.data()), mesh.size());
      mesh_writer_->bytes_written(mesh.size());
    }
  }
}

bool PostureMeshMerge::connect(std::string shmdata_socket_path) {
  unique_lock<mutex> connectLock(connect_mutex_);

  int index = source_id_;
  source_id_ += 1;
  int shmreader_id = shmreader_id_;
  shmreader_id_++;

  auto reader = std::make_unique<ShmdataFollower>(
      this,
      shmdata_socket_path,
      [=](void* data, size_t size) {
        mutex_.lock();
        auto typeIt = mesh_readers_caps_.find(shmreader_id);
        if (typeIt == mesh_readers_caps_.end()) {
          mutex_.unlock();
          return;
        }
        string type = typeIt->second;
        mutex_.unlock();

        if (merger_ == nullptr || (type != string(POLYGONMESH_TYPE_BASE))) return;

        merger_->setInputMesh(index, vector<uint8_t>((uint8_t*)data, (uint8_t*)data + size));

        bool already_updated = cameras_updated_[index];
        cameras_updated_[index] = true;
        if (already_updated || all(cameras_updated_)) {
          zero(cameras_updated_);
          update_wanted_ = true;
          update_cv_.notify_one();
        }
      },
      [=](string caps) {
        unique_lock<mutex> lock(mutex_);
        mesh_readers_caps_[shmreader_id] = caps;
      });

  mesh_readers_[shmdata_socket_path] = std::move(reader);
  return true;
}

bool PostureMeshMerge::disconnect(std::string) {
  std::lock_guard<mutex> lock(mutex_);
  return true;
}

bool PostureMeshMerge::disconnect_all() {
  source_id_ = 0;
  return true;
}

bool PostureMeshMerge::all(const vector<bool>& status) {
  for (auto s : status)
    if (!s) return false;

  return true;
}

void PostureMeshMerge::zero(vector<bool>& status) {
  for (uint32_t i = 0; i < status.size(); ++i) status[i] = false;
}

bool PostureMeshMerge::can_sink_caps(std::string caps) { return (caps == POLYGONMESH_TYPE_BASE); }

}  // namespace switcher
