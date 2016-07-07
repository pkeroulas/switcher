#include "./posture_directreconstruct.hpp"

#include <functional>
#include <iostream>
#include <regex>
#include <snappy.h>

#include <pcl/io/obj_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/registration/transforms.h>
#include <boost/make_shared.hpp>

#include "switcher/scope-exit.hpp"

using namespace std;
using namespace pcl;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureDirectReconstruct,
                                     "posturedirectreconstruct",
                                     "Depth images to textured mesh",
                                     "video",
                                     "reader/writer",
                                     "Create a mesh with texture coordinates "
                                     "from composite depth images from "
                                     "multiple cameras",
                                     "LGPL",
                                     "Emmanuel Durand and Sebastien Paquet");

PostureDirectReconstruct::PostureDirectReconstruct(const std::string&)
    : shmcntr_(static_cast<Quiddity*>(this)) {
  calibration_reader_ = make_unique<CalibrationReader>("default.kvc");
  register_ = make_unique<Register>();
}

PostureDirectReconstruct::~PostureDirectReconstruct() {}

// Basically what we do here is: 
// 1) decompress and extract received depth images
// 2) feed those into DirectMesher (instead of SolidifyGPU) to get textured meshes
// 3) shift each texturedmesh's coordinates depending on the camera it came from -- new_u = (old_u + camNo)/3
// 4) write out the texturedmesh into a shmData for consumption by Blender

bool PostureDirectReconstruct::set_dimensions_and_decompression(string caps) {
  unsigned int width, stackedHeight, height;

  regex regCompress, regVideo, regFormat;
  regex regNcams, regWidth, regHeight;
  smatch match;

  try {
    // regVideo  = regex("(.*application/x-composite-zcam16c)(.*)",
    // regex_constants::extended);
    regCompress = regex("(.*posture/compressed-depth)(.*)", regex_constants::extended);
    regVideo = regex("(.*video/x-raw)(.*)", regex_constants::extended);
    regFormat = regex("(.*format=\\(string\\))(.*)", regex_constants::extended);
    regNcams  = regex("(.*nCams=\\(int\\))(.*)", regex_constants::extended);
    regWidth  = regex("(.*width=\\(int\\))(.*)", regex_constants::extended);
    regHeight = regex("(.*height=\\(int\\))(.*)",regex_constants::extended);
  } catch (const regex_error& e) {
    cout << "PostureDirectReconstruct::" << __FUNCTION__ << " - Regex error code: " << e.code()
         << endl;
    return false;
  }

  decompress_ = regex_match(caps, regCompress);

  // check prefix
  if (!decompress_ && !regex_match(caps, regVideo)) return false;

  // check format
  if (regex_match(caps, match, regFormat)) {
    string subMatch = match[2].str();
    subMatch = subMatch.substr(0, subMatch.find(","));
    if ("GRAY16_BE" != subMatch)
      return false;
  }

  // get number of cameras
  if (regex_match(caps, match, regNcams)) {
    string subMatch = match[2].str();
    camera_nbr_ = stoi(subMatch.substr(0, subMatch.find(",")));
  } else return false;

  // get width
  if (regex_match(caps, match, regWidth)) {
    string subMatch = match[2].str();
    width = stoi(subMatch.substr(0, subMatch.find(",")));
  } else return false;

  // get total height
  if (regex_match(caps, match, regHeight)) {
    string subMatch = match[2].str();
    stackedHeight = stoi(subMatch.substr(0, subMatch.find(",")));
    height = stackedHeight / camera_nbr_;
  } else return false;

  depthImages_.resize(camera_nbr_);
  depthImages_dims_.resize(camera_nbr_);

  depthImages_dims_.clear();
  for (int i = 0; i < camera_nbr_; ++i)
    depthImages_dims_.emplace_back(vector<uint32_t>({width,height}));

  return true;
}

bool PostureDirectReconstruct::connect(std::string shmdata_socket_path) {
  unique_lock<mutex> connectLock(connect_mutex_);

  depthDataReader_.reset();
  depthDataReader_ = make_unique<ShmdataFollower>(
      this,
      shmdata_socket_path,
      [&](void* data, size_t size) {

        if (depthImages_.size() == 0)  // not ready to process yet (dimensions not set)
          return;

        vector<char> uncompressedData;
        auto rawImageData = reinterpret_cast<char*>(data);

        // if necessary, uncompress the compositeDepthData 
        if (decompress_)
        {
          if (snappy::IsValidCompressedBuffer((char*)data, size))
          {
            string uncompressedBuffer;
            snappy::Uncompress((char*)data, size, &uncompressedBuffer);
            // put result into a vector of char (maybe we can directly point to the string data?
            uncompressedData.resize(uncompressedBuffer.size());
            // unique_lock<mutex> lockRawImageData(rawImageData_mutex_);
            memcpy((void*)uncompressedData.data(), (void*)uncompressedBuffer.data(), uncompressedBuffer.size());
            rawImageData = (char*)uncompressedData.data();
          } else {
            cout << "PostureDirectReconstruct::" << __FUNCTION__ << " Invalid Snappy buffer"
                 << endl;
            return;
          }
        }

        // Split the composite depth image and fill all the depthImages_ buffers
        size_t offsetInBytes = 0;
        for (int i = 0; i < camera_nbr_; ++i) {
          int imgSize = depthImages_dims_[i][0] * depthImages_dims_[i][1];
          int sizeInBytes = imgSize * sizeof(uint16_t);

          unique_lock<mutex> lockDepth(depth_mutex_);
          depthImages_[i].resize(imgSize);
          memcpy(depthImages_[i].data(), rawImageData + offsetInBytes, sizeInBytes);
          // lockDepth.unlock();

          offsetInBytes += sizeInBytes;
        
        }
        
        update_wanted_ = true;
        update_cv_.notify_one();
      },
      [=](const string& caps) {  // 2- caps specifies what kind of data is in the buffer
        // use caps to fill in number of cameras and depth dimensions
        // "video/x-raw,format=(string)GRAY16_BE,width=(int)640,height=(int)960,framerate=30/1,nCams=(int)2"
        set_dimensions_and_decompression(caps);
      });

  // ensure that the reader object persists
  //shmdata_readers_[shmdata_socket_path] = std::move(depthDataReader_);
  return true;
}

bool PostureDirectReconstruct::start() {
  // now that connection has been made
  // and depthDataReader is doing its job to fill depthImages_,
  // let's start the update loop, which actually creates and writes the mesh
  mesh_writer_.reset();

  calibration_reader_->loadCalibration(calibration_path_);
  if (!(*calibration_reader_) ||
      calibration_reader_->getCalibrationParams().size() <
          (uint32_t)camera_nbr_)
    return false;

  // could use make_unique instead
  directMesher_ =
      unique_ptr<posture::DirectMesher>(new posture::DirectMesher(pixelResolution_, angleTolerance_));
  directMesher_->setCalibration(calibration_reader_->getCalibrationParams());

  meshSerializer_= unique_ptr<posture::MeshSerializer>(new posture::MeshSerializer());
  meshSerializer_->setCompress(true);  // to feed faster into local Blender

  update_loop_started_ = true;
  update_thread_ = thread([&]() { update_loop(); });

  return true;
}


bool PostureDirectReconstruct::stop() {
  update_loop_started_ = false;
  unique_lock<mutex> lockUpdate(update_mutex_);

  update_wanted_ = true;
  update_cv_.notify_one();
  if (update_thread_.joinable()) update_thread_.join();

  if (registering_thread_.joinable()) registering_thread_.join();

  // cameras_.clear();
  mesh_writer_.reset();

  return true;
}

void PostureDirectReconstruct::update_loop() {
  // this is the end goal: construct a serialized mesh
  std::vector<uint8_t> mesh_serialized{};

  while (update_loop_started_) {
    unique_lock<mutex> lock(update_mutex_);  // still unsure what this does :)

    if (!update_wanted_) update_cv_.wait(lock);
    update_wanted_ = false;

    if (!update_loop_started_) break;

    if (reload_calibration_) {
      calibration_reader_->reload();
      if (*calibration_reader_ &&
          calibration_reader_->getCalibrationParams().size() >=
              (uint32_t)camera_nbr_) {
        auto calibration = calibration_reader_->getCalibrationParams();
        directMesher_->setCalibration(calibration);
      }
    }

    // is this where we should unlock?
    lock.unlock();

    // TODO-PERF: write this using concurrency
    // cout << "DirectReconstruct: 2. creating multiMesh" << endl;

    // Now create a textured mesh for each depth camera
    vector<TextureMesh::Ptr> multiMesh;
    // Could we do this *after* the unlock?
    for (int camNo = 0; camNo < camera_nbr_; camNo++)
    {
      TextureMesh::Ptr texturedMesh = boost::make_shared<pcl::TextureMesh>();

      // get the cloud and release the lock
      // cout << "DirectReconstruct: 2.1 getting the lock" << endl;
      unique_lock<mutex> lockDepth(depth_mutex_);
      // cout << "DirectReconstruct: 2.2 getting the cloud" << endl;
      int sizeX = depthImages_dims_[camNo][0];
      int sizeY = depthImages_dims_[camNo][1];
      auto cloud = directMesher_->convertToXYZPointCloud(depthImages_[camNo], sizeX, sizeY, camNo); // why provide camNo?
      lockDepth.unlock();

      directMesher_->getMesh(cloud, texturedMesh, camNo);

      // scale the texture coordinates in the texturedMesh
      // for each point in the mesh
      for (auto& uv : texturedMesh->tex_coordinates[0]) uv[0] = (uv[0] + camNo) / camera_nbr_;

      // append to our multimesh
      multiMesh.push_back(texturedMesh);
    }
    // cout << "DirectReconstruct: 3. serializing multiMesh" << endl;

    // serialize the mesh into a form digestible by Blender: MCTM-texMesh
    // for single-polymesh output:
    //  mesh_serialized = meshSerializer_->serialize(multiMesh[0], 0);
    // for multiMesh output:
    mesh_serialized = meshSerializer_->serialize(multiMesh, 0);

    // cout << "DirectReconstruct: 4. writing multiMesh" << endl;
    // time to write the mesh - as long as it's not empty
    if (multiMesh[0]->tex_polygons.size() > 0) {
      // cout << "DirectReconstruct: 5. writing multiMesh" << endl;
      if (!mesh_writer_ || mesh_serialized.size() > mesh_writer_->writer<MPtr(&shmdata::Writer::alloc_size)>()) 
      {
        /// for polymesh output:
        // auto data_type = string(POLYGONMESH_TYPE_BASE);  // TYPE_MULTIMESH);
        /// for multimesh output:
        auto data_type = string(POLYGONMESH_TYPE_MULTIMESH);  // TYPE_MULTIMESH);
        mesh_writer_.reset();
        mesh_writer_ = make_unique<ShmdataWriter>(
            this, make_file_name("mesh"), mesh_serialized.size() * 2, data_type);

        if (!mesh_writer_) {
          g_warning("Unable to create mesh writer");
          return;
        }
      }

      mesh_writer_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(
          reinterpret_cast<char*>(mesh_serialized.data()),
          mesh_serialized.size());
      mesh_writer_->bytes_written(mesh_serialized.size());
      // cout << "DirectReconstruct: written bytes: " << mesh_serialized.size()
    }
  }
}

bool PostureDirectReconstruct::init() {
  init_startable(this);

  shmcntr_.install_connect_method(
      [this](const std::string path) { return connect(path); },
      [this](const std::string path) { return disconnect(path); },
      [this]() { return disconnect_all(); },
      [this](const std::string caps) { return can_sink_caps(caps); },
      8);

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

  register_id_ = pmanage<MPtr(&PContainer::make_parented_bool)>(
      "improve_registration",
      "calibration",
      [this](const bool& val) {
        improve_registering_ = val;
        return true;
      },
      [this]() { return improve_registering_; },
      "Improve registration",
      "Automatically improve cameras calibration, once",
      improve_registering_);

  //
  // DirectMesher parameters
  // 1. pixelResolution_  2. angleTolerance_  3. clippingDepth (tho depth was clipped upon scan)
  pmanage<MPtr(&PContainer::make_group)>(
      "directMesh", "Direct Mesher parameters", "Reconstruction grid parameters");

  pmanage<MPtr(&PContainer::make_parented_int)>("pixel_resolution",
                                                "directMesh",
                                                [this](const int& val) {
                                                  pixelResolution_ = val;
                                                  return true;
                                                },
                                                [this]() { return pixelResolution_; },
                                                "Pixel resolution",
                                                "Spacing between samples taken from the depth map",
                                                pixelResolution_,
                                                1,
                                                40);

  pmanage<MPtr(&PContainer::make_parented_double)>(
      "angle_tolerance",
      "directMesh",
      [this](const double& val) {
        angleTolerance_ = val;
        return true;
      },
      [this]() { return angleTolerance_; },
      "Angular tolerance",
      "Angular tolerance for the direct mesh reconstruction",
      angleTolerance_,
      1.0,
      20.0);

  return true;
}

bool PostureDirectReconstruct::disconnect(std::string) {
  // std::lock_guard<mutex> lock(mutex_);
  return true;
}

bool PostureDirectReconstruct::disconnect_all() {
  // source_id_ = 0;
  return true;
}

bool PostureDirectReconstruct::can_sink_caps(std::string caps) {
  return (caps != "");  // see setDimensions
}

}  // namespace switcher
