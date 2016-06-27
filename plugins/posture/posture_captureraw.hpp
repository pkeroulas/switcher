#ifndef POSTURE_CAPTURERAW_H
#define POSTURE_CAPTURERAW_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "./posture.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

#include <boost/make_shared.hpp>

namespace switcher {
class PostureCaptureRaw : public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureCaptureRaw);
  PostureCaptureRaw(const std::string&);
  PostureCaptureRaw(const PostureCaptureRaw&) = delete;
  ~PostureCaptureRaw();

  bool start();
  bool stop();

 private:
  bool compress_depth_{false};
  std::string calibration_path_{"default.kvc"};

  int camera_nbr_{1};
  Selection capture_modes_enum_{{"Default mode",
                                 "SXGA 15Hz",
                                 "VGA 30Hz",
                                 "VGA 25Hz",
                                 "QVGA 25Hz",
                                 "QVGA 30Hz",
                                 "QVGA 60Hz",
                                 "QQVGA 25Hz",
                                 "QQVGA 30Hz",
                                 "QQVGA 60Hz"},
                                0};

  std::mutex camera_mutex_;
  std::vector<std::unique_ptr<posture::ZCamera>> cameras_{};
  std::unique_ptr<posture::Register> register_{nullptr};
  std::unique_ptr<posture::CalibrationReader> calibration_reader_{nullptr};

  PropertyBase::prop_id_t register_id_;
  bool reload_calibration_{false};

  std::vector<bool> rgbCameras_updated_{};
  std::vector<bool> depthCameras_updated_{};

  // TODO: eventually change into CamDataPackager
  std::unique_ptr<posture::CamDataPackagerImpl> cameraPackager_{nullptr};

  // filtering params
  int bilateral_filter_kernel_size_{5};
  double bilateral_filter_sigma_pos_{5.f};
  double bilateral_filter_sigma_value_{1000.f};
  int bilateral_filter_iterations_{1};
  int hole_filling_kernel_size_{1};
  int hole_filling_iterations_{1};

  // clipping params
  unsigned short clipping_depth_{3000};

  std::atomic_bool update_loop_started_{false};
  std::atomic_bool update_wanted_{false};
  std::mutex update_mutex_;
  std::condition_variable update_cv_;
  std::thread update_thread_;

  std::unique_ptr<ShmdataWriter> depth_writer_{nullptr};
  std::unique_ptr<ShmdataWriter> texture_writer_{nullptr};

  bool init() final;
  void update_loop();

  bool all(const std::vector<bool>& status);
  void zero(std::vector<bool>& status);
  void cb_frame_depth(int index,
                      std::vector<unsigned char>& depth,
                      int width,
                      int height);
  void cb_frame_RGB(int index,
                    std::vector<unsigned char>& rgb,
                    int width,
                    int height);
};

SWITCHER_DECLARE_PLUGIN(PostureCaptureRaw);
}  // namespace switcher
#endif  // POSTURE_CAPTURERAW_H
