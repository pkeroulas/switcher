#ifndef POSTURE_DIRECTRECONSTRUCT_H
#define POSTURE_DIRECTRECONSTRUCT_H

#include <atomic>
#include <condition_variable>
#include <iostream>
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

class ConsoleLogger: public shmdata::AbstractLogger
{
    private:
        void on_error(std::string &&str) final { std::cout << "Shmdata::ConsoleLogger - " << str << std::endl; }
        void on_critical(std::string &&str) final { std::cout << "Shmdata::ConsoleLogger - " << str << std::endl; }
        void on_warning(std::string &&str) final { std::cout << "Shmdata::ConsoleLogger - " << str << std::endl; }
        void on_message(std::string &&str) final { std::cout << "Shmdata::ConsoleLogger - " << str << std::endl; }
        void on_info(std::string &&str) final { std::cout << "Shmdata::ConsoleLogger - " << str << std::endl; }
        void on_debug(std::string &&str) final { std::cout << "Shmdata::ConsoleLogger - " << str << std::endl; }
};


class PostureDirectReconstruct : public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureDirectReconstruct);
  PostureDirectReconstruct(const std::string&);
  PostureDirectReconstruct(const PostureDirectReconstruct&) = delete;
  ~PostureDirectReconstruct();

  bool start();
  bool stop();

 private:
  ShmdataConnector shmcntr_;
  std::string calibration_path_{"default.kvc"};
  bool setDimensionsAndDecompression(std::string caps);

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
  std::mutex depth_mutex_;
  std::unique_ptr<posture::CalibrationReader> calibration_reader_{nullptr};
  std::unique_ptr<posture::Register> register_{nullptr};
  std::unique_ptr<posture::DirectMesher> directMesher_{nullptr};
  std::unique_ptr<posture::MeshSerializer> meshSerializer_{nullptr};

  PropertyBase::prop_id_t register_id_;
  bool reload_calibration_{false};
  bool improve_registering_{false};
  std::thread registering_thread_{};
  std::atomic_bool is_registering_{false};

  std::vector<std::vector<uint16_t>> compositeDepthImage_{};
  std::vector<std::vector<uint16_t>> depthImages_{};
  std::vector<std::vector<uint32_t>> depthImages_dims_{};

  bool decompress_;

  int pixelResolution_;
  double angleTolerance_;

  std::atomic_bool update_loop_started_{false};
  std::atomic_bool update_wanted_{false};
  std::mutex update_mutex_;
  std::condition_variable update_cv_;
  std::thread update_thread_;

  std::unique_ptr<ShmdataFollower> depthDataReader_{};
  std::mutex connect_mutex_{};
  std::unique_ptr<ShmdataWriter> mesh_writer_{nullptr};

  bool init() final;
  void update_loop();
  void reset_solidify();

  bool all(const std::vector<bool>& status);
  void zero(std::vector<bool>& status);
  void cb_frame_depth(int index,
                      std::vector<unsigned char>& depth,
                      int width,
                      int height);

  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string shmName);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);
};

SWITCHER_DECLARE_PLUGIN(PostureDirectReconstruct);
}  // namespace switcher
#endif  // POSTURE_DIRECTRECONSTRUCT_H
