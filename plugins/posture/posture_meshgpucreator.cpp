#include "./posture_meshgpucreator.hpp"
#include "switcher/std2.hpp"

#include <functional>
#include <iostream>

//#include <boost/make_shared.hpp>
#include <pcl/io/obj_io.h>

using namespace std;
using namespace posture;

namespace switcher {
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION (
					PostureMeshGPUCreator,
					"posturescansrcgpu",
					"Scan 3D GPU",
					"video",
					"writer",
					"Grab mesh using zcameras on GPU",
					"LGPL",
					"Ludovic Schreiber");

  PostureMeshGPUCreator::PostureMeshGPUCreator (const std::string &)
  {
    init_startable (this);
    mesh_creator_ = std::unique_ptr<DepthMapToMesh> (new DepthMapToMesh ());

    pmanage<MPtr(&PContainer::make_int)>
            ("camera_number",
             [this](const int &val)
             {
                nbr_cam_ = val;
                cameras_.resize(nbr_cam_);
                mesh_creator_->setDepthMapNbr(nbr_cam_);
                auto index = 0;
                for (; index < nbr_cam_; index++)
                {
                    cameras_[index] = std2::make_unique<ZCamera>();
                    cameras_[index]->setDeviceIndex(index);
                    cameras_[index]->setCaptureMode(ZCamera::CaptureMode_QQVGA_30Hz);
                    cameras_[index]->
                    setCallbackDepth([this, index] (void*,
                                                   std::vector<unsigned char>& depth,
                                                   int width,
                                                   int height) -> void
                    {
                        cb_frame_depth(index, depth, width, height);
                    }, nullptr);
                }
                return true;
             },
             [this](){return nbr_cam_;},
             "Camera number",
             "Number of used cameras",
             nbr_cam_,
             1,
             9);
    
    pmanage<MPtr(&PContainer::make_string)>
            ("calibration_path",
             [this](const std::string &val)
             {
                calibration_path_ = val;
                mesh_creator_ -> setCalibrationPath (calibration_path_);
                return true;
             },
             [this](){return calibration_path_;},
             "calibration_path",
             "Path to the calibration file",
             calibration_path_);

    pmanage<MPtr(&PContainer::make_string)>
            ("devices_path",
             [this](const std::string &val)
             {
                devices_path_ = val;
                return true;
             },
             [this](){return devices_path_;},
             "devices_path",
             "Path to the devices description file",
             devices_path_);

    pmanage<MPtr(&PContainer::make_bool)>
            ("save_mesh",
             [this](bool val)
             {
                save_mesh_ = val;
                mesh_creator_->setSaveMesh (save_mesh_);
                return true;
             },
             [this](){return save_mesh_;},
             "save_mesh",
             "Save the current mesh if true",
             save_mesh_);

    pmanage<MPtr(&PContainer::make_int)>
            ("grid resolution",
             [this](int val){
             if (val >= 3)
             {
                resolution_ = val;
                mesh_creator_->setGridResolution(resolution_);
             }
             return true;
             },
             [this](){return resolution_;},
             "grid resolution",
             "resolution for the mesh reconstruction",
             resolution_,
             3,
             9);

    pmanage<MPtr(&PContainer::make_double)>
            ("grid size",
             [this](double val)
             {
                size_ = val;
                mesh_creator_->setGridSize(size_);
                return true;
             },
             [this](){return size_;},
             "grid size",
             "size of the cube for the mesh reconstruction",
             size_,
             0.1f,
             99.0f);

    // We want 1 camera at construction, fix this:
    // cameras_.resize(1);
    // cameras_ [0] = std::shared_ptr<ZCamera> (new ZCamera ());
    // set_number_camera(nbr_cam_, this);
  }

  PostureMeshGPUCreator::~PostureMeshGPUCreator ()
  {
    stop();
  }

  bool
  PostureMeshGPUCreator::start()
  {
    std::lock_guard<std::mutex> lock (mutex_);
    auto index = 0;
    for (; index < nbr_cam_; index++)
    {
      cameras_[index] -> start ();
    }

    is_started_ = true;
    return true;
  }

  bool
  PostureMeshGPUCreator::stop()
  {
    std::lock_guard<std::mutex> lock (mutex_);
    auto index = 0;
    for (; index < nbr_cam_; index++)
    {
      cameras_ [index] -> stop ();
    }

    mesh_writer_.reset();
    is_started_ = false;
    return true;
  }

  bool PostureMeshGPUCreator::init()
  {
    return true;
  }

  void
  PostureMeshGPUCreator::cb_frame_depth (int index, std::vector<unsigned char>& depth, int width, int height){
    mesh_creator_->setInputDepthMap(index, depth, width, height);
    mesh_creator_->getMesh(output_);

    if (!mesh_writer_ || output_.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>())
    {
      mesh_writer_.reset();
      mesh_writer_ = std2::make_unique<ShmdataWriter>(this,
						      make_file_name("mesh"),
						      output_.size() * 2,
						      string(POLYGONMESH_TYPE_BASE));
      if (!mesh_writer_)
      {
        g_warning("Unable to create mesh callback");
        return;
      }
    }
    mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(const_cast<unsigned char*>(output_.data()), 
							      output_.size());
    mesh_writer_->bytes_written(output_.size());
  }

}  // namespace switcher
