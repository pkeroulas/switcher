#include "./posture_captureraw.hpp"

#include <functional>
#include <iostream>

#include <pcl/io/obj_io.h>
#include <boost/make_shared.hpp>

#include "switcher/scope-exit.hpp"

using namespace std;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    PostureCaptureRaw,
    "posturecaptureraw",
    "capture and package RGBD",
    "video",
    "writer/device",
    "Create composite texture and depth maps from rgbd cameras",
    "LGPL",
    "Sebastien Paquet");

PostureCaptureRaw::PostureCaptureRaw(const std::string&) {
  calibration_reader_ = make_unique<CalibrationReader>("default.kvc");
  register_ = make_unique<Register>();
}

PostureCaptureRaw::~PostureCaptureRaw() {}

bool PostureCaptureRaw::start() {
  cameras_.clear();
  depth_writer_.reset();
  texture_writer_.reset();

  //images_.resize(camera_nbr_);
  //images_dims_.resize(camera_nbr_);
  rgbCameras_updated_.resize(camera_nbr_);
  depthCameras_updated_.resize(camera_nbr_);

  cameraPackager_ = make_unique<CamDataPackagerImpl>(camera_nbr_);

  calibration_reader_->loadCalibration(calibration_path_);
  if (!(*calibration_reader_) ||
      calibration_reader_->getCalibrationParams().size() <
          (uint32_t)camera_nbr_)
    return false;

  for (int i = 0; i < camera_nbr_; ++i) {
    cameras_.emplace_back(new posture::ZCamera());
    cameras_.back()->setCaptureMode(
        (posture::ZCamera::CaptureMode)capture_modes_enum_.get());
    cameras_.back()->setCalibration(
        calibration_reader_->getCalibrationParams()[i]);
    cameras_.back()->setDeviceIndex(i);

    // cameras_.back()->setCallbackCloud(
    //     [=](void*, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
    //       cb_frame_cloud(i, cloud);
    //     },
    //     nullptr);

    cameras_.back()->setCallbackDepth(
        [=](void*, std::vector<uint8_t>& depth, int width, int height) {
          cb_frame_depth(i, depth, width, height);
        },
        nullptr);

    cameras_.back()->setCallbackRgb(
        [=](void*, std::vector<uint8_t>& rgb, int width, int height) {
          cb_frame_RGB(i, rgb, width, height);
        },
        nullptr);
  }

  update_loop_started_ = true;
  update_thread_ = thread([&]() { update_loop(); });

  return true;
}

bool PostureCaptureRaw::stop() {
  update_loop_started_ = false;
  unique_lock<mutex> lockUpdate(update_mutex_);
  update_cv_.notify_one();
  if (update_thread_.joinable()) update_thread_.join();

  // if (registering_thread_.joinable()) registering_thread_.join();

  cameras_.clear();
  cameraPackager_.reset();
  depth_writer_.reset();
  texture_writer_.reset();

  return true;
}

void PostureCaptureRaw::update_loop() {
  for (auto& cam : cameras_) {
    cam->start();
    if (!cam->isReady()) return;
  }

  while (update_loop_started_) {
    unique_lock<mutex> lock(update_mutex_);
    if (!update_wanted_) update_cv_.wait(lock);

    update_wanted_ = false;

    if (!update_loop_started_) break;

    // // The registerer runs in a separate thread and is updated at
    // // its own pace
    // if (improve_registering_ && !is_registering_) {
    //   if (registering_thread_.joinable()) registering_thread_.join();

    //   is_registering_ = true;
    //   registering_thread_ = thread([=]() {
    //     calibration_reader_->reload();
    //     if (*calibration_reader_ &&
    //         calibration_reader_->getCalibrationParams().size() >=
    //             (uint32_t)camera_nbr_) {
    //       auto calibration = calibration_reader_->getCalibrationParams();

    //       register_->setGuessCalibration(calibration);
    //       calibration = register_->getCalibration();

    //       // The previous call can take some time, so we check again that
    //       // automatic registration is active
    //       if (improve_registering_) {
    //         solidifyGPU_->setCalibration(calibration);
    //         colorize_->setCalibration(calibration);
    //       }
    //     }

    //     is_registering_ = false;
    //     pmanage<MPtr(&PContainer::set<bool>)>(register_id_, false);
    //   });
    // }

    if (reload_calibration_) {
      calibration_reader_->reload();
      if (*calibration_reader_ &&
          calibration_reader_->getCalibrationParams().size() >=
              (uint32_t)camera_nbr_) {
        auto calibration = calibration_reader_->getCalibrationParams();
      }
    }

    /////////////////////////////////////////////
    // get the composite texture and depth data from the capturer
    
    // solidifyGPU_->getMesh(mesh);

    lock.unlock();

    unique_lock<mutex> lockCamera(camera_mutex_);
    //colorize_->setInput(mesh, images_, images_dims_);
    lockCamera.unlock();

    // colorize_->getTexturedMesh(texturedMesh);
    // colorize_->getTexturedMesh(mesh_serialized);

    uint32_t texture_width, texture_height;
    std::vector<unsigned char> compositeTexture = cameraPackager_->getCompositeTexture(texture_width, texture_height);
    std::vector<std::vector<uint32_t>> depth_dims;
    std::vector<unsigned char> compositeDepth = cameraPackager_->getCompositeDepth(depth_dims);

    // write out to shmdata
    // NOTE: we assume all cameras have the same dimensions as camera #0
    if (compositeTexture.size() > 0 && compositeDepth.size() > 0) 
    {
      if (!depth_writer_ ||
          compositeDepth.size() > depth_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) 
      {
        auto data_type = string(COMPOSITE_DEPTHMAP_16BITS_TYPE_COMPRESSED); // defined in ./posture.hpp
        depth_writer_.reset();
        depth_writer_ = make_unique<ShmdataWriter>(
            this,
            make_file_name("depth"),
            compositeDepth.size() * 2, string(compress_depth_? "compressed " : "") +
            "video/x-raw,format=(string)GRAY16_BE,width=(int)" + to_string(depth_dims[0][0]) +
            ",height=(int)" + to_string(camera_nbr_ * depth_dims[0][1]) + 
            ",framerate=30/1,nCams=(int)" + to_string(camera_nbr_));

        if (!depth_writer_) {
          g_warning("Unable to create depth writer");
          return;
        }
      }

      depth_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
          reinterpret_cast<char*>(compositeDepth.data()), compositeDepth.size());
      depth_writer_->bytes_written(compositeDepth.size() * sizeof(unsigned char));

      if (!texture_writer_ ||
          compositeTexture.size() >
              texture_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) {
        auto data_type = string(COMPOSITE_RGB_TYPE_BASE); // unused for now...
        texture_writer_.reset();
        texture_writer_ = make_unique<ShmdataWriter>(
            this,
            make_file_name("texture"),
            compositeTexture.size() * 2,
            "video/x-raw,format=(string)BGR,width=(int)" + to_string(texture_width) +
                ",height=(int)" + to_string(texture_height) + ",framerate=30/1");

        if (!texture_writer_) {
          g_warning("Unable to create texture writer");
          return;
        }
      }

      texture_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
          reinterpret_cast<char*>(compositeTexture.data()), compositeTexture.size());
      texture_writer_->bytes_written(compositeTexture.size());
    }
  }
}

bool PostureCaptureRaw::init() {
  init_startable(this);

  pmanage<MPtr(&PContainer::make_selection)>(
      "capture_mode",
      [this](const size_t& val) {
        capture_modes_enum_.select(val);
        return true;
      },
      [this]() { return capture_modes_enum_.get(); },
      "Capture mode",
      "Capture mode for the cameras",
      capture_modes_enum_);

  pmanage<MPtr(&PContainer::make_int)>("camera_nbr",
                                       [this](const int& val) {
                                         camera_nbr_ = std::max(1, val);
                                         return true;
                                       },
                                       [this]() { return camera_nbr_; },
                                       "Number of cameras",
                                       "Number of cameras to grab from",
                                       camera_nbr_,
                                       1,
                                       7);

  pmanage<MPtr(&PContainer::make_bool)>(
      "compress_depth_",
      [this](const bool& val) {
        compress_depth_ = val;
        if (cameraPackager_) cameraPackager_->setCompressDepth(compress_depth_);
        return true;
      },
      [this]() { return compress_depth_; },
      "Compress depth",
      "Compress the composite depth image",
      compress_depth_);

  //  
  // Calibration
  pmanage<MPtr(&PContainer::make_group)>(
      "calibration", "Calibration", "Camera calibration parameters");

  pmanage<MPtr(&PContainer::make_parented_string)>(
      "calibration_path",
      "calibration",
      [this](const std::string& val) {
        calibration_path_ = val;
        return true;
      },
      [this]() { return calibration_path_; },
      "Calibration path",
      "Path to the calibration file",
      calibration_path_);

  register_id_ = pmanage<MPtr(&PContainer::make_parented_bool)>(
      "reload_calibration",
      "calibration",
      [this](const bool& val) {
        reload_calibration_ = val;
        return true;
      },
      [this]() { return reload_calibration_; },
      "Reload calibration",
      "Reload the calibration from the given file",
      reload_calibration_);
  //
  // Filtering
  pmanage<MPtr(&PContainer::make_group)>("filtering", "Filtering", "Filtering");

  pmanage<MPtr(&PContainer::make_parented_int)>("bilateral_filter_kernel_size",
                                                "filtering",
                                                [this](const int& val) {
                                                  bilateral_filter_kernel_size_ = val;
                                                  for (auto& zcamera_: cameras_)
                                                    zcamera_->setBilateralFiltering(
                                                        bilateral_filter_kernel_size_,
                                                        bilateral_filter_sigma_pos_,
                                                        bilateral_filter_sigma_value_,
                                                        bilateral_filter_iterations_);
                                                  return true;
                                                },
                                                [this]() { return bilateral_filter_kernel_size_; },
                                                "Filter kernel size",
                                                "Depth map filter kernel size",
                                                bilateral_filter_kernel_size_,
                                                1,
                                                32);

  pmanage<MPtr(&PContainer::make_parented_float)>("bilateral_filter_sigma_pos_",
                                                  "filtering",
                                                  [this](const float& val) {
                                                    bilateral_filter_sigma_pos_ = val;
                                                    for (auto& zcamera_: cameras_)
                                                      zcamera_->setBilateralFiltering(
                                                          bilateral_filter_kernel_size_,
                                                          bilateral_filter_sigma_pos_,
                                                          bilateral_filter_sigma_value_,
                                                          bilateral_filter_iterations_);
                                                    return true;
                                                  },
                                                  [this]() { return bilateral_filter_sigma_pos_; },
                                                  "Filter spatial sigma",
                                                  "Depth map filter spatial sigma",
                                                  bilateral_filter_sigma_pos_,
                                                  0.1,
                                                  16.0);

  pmanage<MPtr(&PContainer::make_parented_float)>(
      "bilateral_filter_sigma_value_",
      "filtering",
      [this](const float& val) {
        bilateral_filter_sigma_value_ = val;
        for (auto& zcamera_: cameras_)
          zcamera_->setBilateralFiltering(bilateral_filter_kernel_size_,
                                          bilateral_filter_sigma_pos_,
                                          bilateral_filter_sigma_value_,
                                          bilateral_filter_iterations_);
        return true;
      },
      [this]() { return bilateral_filter_sigma_value_; },
      "Filter value sigma",
      "Depth map filter value sigma",
      bilateral_filter_sigma_value_,
      1.0,
      2000.0);

  pmanage<MPtr(&PContainer::make_parented_int)>("bilateral_filter_iterations",
                                                "filtering",
                                                [this](const int& val) {
                                                  bilateral_filter_iterations_ = val;
                                                  for (auto& zcamera_: cameras_)
                                                    zcamera_->setBilateralFiltering(
                                                        bilateral_filter_kernel_size_,
                                                        bilateral_filter_sigma_pos_,
                                                        bilateral_filter_sigma_value_,
                                                        bilateral_filter_iterations_);
                                                  return true;
                                                },
                                                [this]() { return bilateral_filter_iterations_; },
                                                "Filter iterations",
                                                "Depth map filter iteration count",
                                                bilateral_filter_iterations_,
                                                1,
                                                8);

  pmanage<MPtr(&PContainer::make_parented_int)>(
      "hole_filling_kernel_size",
      "filtering",
      [this](const int& val) {
        hole_filling_kernel_size_ = val;
        for (auto& zcamera_: cameras_)
          zcamera_->setHoleFiltering(hole_filling_kernel_size_, hole_filling_iterations_);
        return true;
      },
      [this]() { return hole_filling_kernel_size_; },
      "Hole filling kernel size",
      "Hole filling kernel size",
      hole_filling_kernel_size_,
      0,
      4);

  pmanage<MPtr(&PContainer::make_parented_int)>(
      "hole_filling_iterations",
      "filtering",
      [this](const int& val) {
        hole_filling_iterations_ = val;
        for (auto& zcamera_: cameras_)
          zcamera_->setHoleFiltering(hole_filling_kernel_size_, hole_filling_iterations_);
        return true;
      },
      [this]() { return hole_filling_iterations_; },
      "Hole filling iterations",
      "Hole filling iterations",
      hole_filling_iterations_,
      0,
      8);

  //
  // Depth clipping
  pmanage<MPtr(&PContainer::make_int)>("clipping_depth",
                                       [this](const int& val) {
                                         clipping_depth_ = std::min(5000, val);
                                         return true;
                                       },
                                       [this]() { return clipping_depth_; },
                                       "Depth limit",
                                       "Anything farther than the depth limit will be ignored",
                                       clipping_depth_,
                                       1,
                                       5000);

  return true;
}


bool PostureCaptureRaw::all(const vector<bool>& status) {
  for (auto s : status)
    if (!s) return false;

  return true;
}

void PostureCaptureRaw::zero(vector<bool>& status) {
  for (uint32_t i = 0; i < status.size(); ++i) status[i] = false;
}

void PostureCaptureRaw::cb_frame_depth(int index,
                                      std::vector<unsigned char>& depth,
                                      int width,
                                      int height) {
  unique_lock<mutex> lockCamera(update_mutex_, std::try_to_lock);
  if (lockCamera.owns_lock()) {

    // convert depth buffer to proper data type: unsigned short
    // TODO: make this work regardless of endianness
    unsigned short *depthdata = reinterpret_cast<unsigned short*>(depth.data());
    std::vector<unsigned short> depth16(depthdata, depthdata + depth.size()/2); // size is halved

    // clip the depth buffer
    for (auto& k: depth16)
        if (k > clipping_depth_) 
            k=0;

    cameraPackager_->StoreDepth(index, depth16, width, height);

    bool already_updated = depthCameras_updated_[index];
    depthCameras_updated_[index] = true;
    if ((already_updated || all(depthCameras_updated_)) &&
        all(rgbCameras_updated_)) {
      zero(depthCameras_updated_);
      update_wanted_ = true;
      update_cv_.notify_one();
    }
  }
}

void PostureCaptureRaw::cb_frame_RGB(int index,
                                    std::vector<unsigned char>& rgb,
                                    int width,
                                    int height) {
  unique_lock<mutex> lock(camera_mutex_);
  cameraPackager_->StoreRGB(index, rgb, width, height);
  rgbCameras_updated_[index] = true;
}

}  // namespace switcher
