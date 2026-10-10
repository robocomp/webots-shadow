/*
 *    Copyright (C) 2026 by RoboLab at the University of Extremadura
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify it under
 *    the terms of the GNU General Public License as published by the Free
 *    Software Foundation, either version 3 of the License, or (at your option)
 *    any later version. See <http://www.gnu.org/licenses/>.
 */
#pragma once
// MediaPlane — the bridge AS the sensor drivers' producer on the zero-copy DDS media plane.
//
// WHY THIS EXISTS. In simulation the chain used to be  Webots → this bridge → Ice (TCP loopback) →
// {lidar3d_dds ×2, zed_camera, ricoh_omni_dds, imu_dds} → DDS. In sim those drivers did no processing
// of their own beyond lidar3d_dds's self-filter (measured 2026-10-10: the ZED frame was 8.0 ms old on
// arrival at zed_camera, which then spent 0.43 ms; the DDS hop itself is 0.3-0.8 ms). Every millisecond
// of that age was the Ice hop and the copies around it. Publishing here removes the hop, the copies and
// five processes, and the drivers are then needed ONLY on the real robot.
//
// WHAT IT KEEPS IDENTICAL, so no consumer can tell the difference:
//   * topics, types, stream ids, formats, QoS and stamps of every stream the drivers published;
//   * the LiDAR cloud in the DEVICE frame exactly as lidar3d_dds built it (axis swap x'=-y, y'=x),
//     with the same Embree self-filter (its sources are copied verbatim from lidar3d_dds);
//   * the five MediaPlaneDDS descriptor endpoints on the drivers' ports, so robot_concept adopts the
//     streams without a config change (see main.cpp).
// WHAT IT CHANGES ON PURPOSE:
//   * the LiDAR ring table is COMPUTED from the device's own geometry instead of clustered from data:
//     in sim the elevations are known exactly, and the table matches the rings actually in the cloud
//     (lidar3d_dds waited for SensorModel.Rings clusters, which the bpearl's half-space cut never
//     delivers, so its table was never advertised);
//   * the LiDAR stamp's simTimestamp/period use the LiDAR period (the bridge used the camera's).
//
// THREADING. Camera, 360 and IMU publish on the Webots thread straight into the DDS loan (a BGRA→RGB
// conversion and a memcpy; Webots image buffers die at the next step()). Each LiDAR has its own worker
// thread: the range image is copied out (≤ 230 KB) and the conversion + self-filter run off the
// Webots thread, so the filter's ray casts never delay robot->step(). Every publisher is driven by
// exactly one thread (media_transport publishers are not thread-safe per instance).

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "media_transport.h"

class ConfigLoader;
class MeshFilter;
namespace webots { class Camera; class RangeFinder; class Lidar; }

namespace shadow_dds
{

// One LiDAR scan as the Webots thread hands it over: the raw range image plus the geometry needed to
// turn it into points. Everything is copied — Webots owns the range image only until the next step.
struct LidarScan
{
    std::vector<float> range;             // layers × columns, row 0 = TOP layer (Webots convention)
    int    columns = 0, layers = 0;
    double fov = 0.0, vfov = 0.0;         // rad
    std::int64_t stamp_ms = 0;            // acquisition instant, wall epoch ms
};

class LidarStream
{
public:
    struct Config
    {
        std::string name;                 // "helios" | "bpearl" — the DSR node and the channel-window key
        std::string topic;
        rc::media::PublisherConfig pub;
        double elev_min_deg = -1e9, elev_max_deg = 1e9;   // [LidarChannels.<name>] window
        bool   up_half_only = false;      // bpearl: keep theta < pi (the downward dome once inverted)
        rc::media::SensorModel model;     // descriptor model; rings/elevations/fov filled from geometry
    };

    LidarStream() = default;
    ~LidarStream();
    LidarStream(const LidarStream&) = delete;
    LidarStream& operator=(const LidarStream&) = delete;

    // `filter` may be null (self-filter disabled). Starts the worker thread.
    bool init(Config cfg, std::unique_ptr<MeshFilter> filter);
    // Webots thread: hand over the newest scan. A scan still waiting is REPLACED, never queued —
    // a consumer acting on a stale cloud is worse than one that skips a beat.
    void submit(LidarScan&& scan);
    [[nodiscard]] std::string descriptor() const;
    void stop();

private:
    void run();
    void process(const LidarScan& scan);
    void set_geometry(const LidarScan& scan, const std::vector<char>& kept, const std::vector<float>& elev_deg);

    Config cfg_;
    std::unique_ptr<MeshFilter> filter_;
    rc::media::LidarPublisher pub_;
    std::uint64_t frame_id_ = 0;
    bool ready_ = false;

    std::thread worker_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::optional<LidarScan> pending_;
    bool stop_ = false;

    mutable std::mutex desc_mtx_;
    std::string descriptor_;
    bool geometry_known_ = false;          // worker thread only
    std::vector<float> xyz_;               // worker scratch
    std::uint64_t published_ = 0, dropped_ = 0;
};

class MediaPlane
{
public:
    MediaPlane();
    ~MediaPlane();

    // Reads [MediaPlane], [Lidar.<name>], [MeshFilter], [LidarChannels.<name>] from the bridge config.
    void init(const ConfigLoader& cfg);

    // Webots thread only. Each returns false when its stream is disabled or the loan failed.
    bool publish_zed(webots::Camera* cam, webots::RangeFinder* ranger, std::int64_t stamp_ms);
    bool publish_360(webots::Camera* cam1, webots::Camera* cam2, std::int64_t stamp_ms);
    bool publish_imu(const rc::media::ImuFrame& sample);
    void submit_lidar(const std::string& name, webots::Lidar* lidar, std::int64_t stamp_ms);

    // Any thread (Ice dispatch): the descriptor a MediaPlaneDDS endpoint returns. "" = not publishing.
    [[nodiscard]] std::string descriptor(const std::string& stream) const;

    void stop();

private:
    rc::media::PublisherConfig image_pub_cfg_, imu_pub_cfg_;
    std::string zed_rgb_topic_, zed_depth_topic_, ricoh_topic_, imu_topic_;
    rc::media::MediaPublisher   zed_rgb_, zed_depth_;
    rc::media::Image360Publisher ricoh_;
    rc::media::ImuPublisher     imu_;
    bool zed_ok_ = false, ricoh_ok_ = false, imu_ok_ = false;
    std::uint64_t zed_rgb_id_ = 0, zed_depth_id_ = 0, ricoh_id_ = 0, imu_id_ = 0;
    // IMU liveness for its descriptor: imu_dds advertised nothing until a sample had gone out
    // recently, so robot_concept never adopts a silent producer. Same contract here.
    std::atomic<std::int64_t> imu_last_wall_ms_{0};
    // Ice dispatch threads query descriptor() from the moment the adapters are up, which is BEFORE the
    // Qt thread runs initialize() and so init(). Release/acquire on this flag is what makes the
    // writers and the LiDAR streams visible to them; until then every descriptor is "".
    std::atomic<bool> initialized_{false};
    std::unique_ptr<LidarStream> helios_, bpearl_;
};

}  // namespace shadow_dds
