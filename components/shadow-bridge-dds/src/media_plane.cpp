/*
 *    Copyright (C) 2026 by RoboLab at the University of Extremadura
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify it under
 *    the terms of the GNU General Public License as published by the Free
 *    Software Foundation, either version 3 of the License, or (at your option)
 *    any later version. See <http://www.gnu.org/licenses/>.
 */
#include "media_plane.h"
#include "mesh_filter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <print>

#include <ConfigLoader/ConfigLoader.h>
#include <opencv2/imgproc.hpp>
#include <webots/Camera.hpp>
#include <webots/Lidar.hpp>
#include <webots/RangeFinder.hpp>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace shadow_dds
{
namespace
{
std::int64_t wall_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
}

template <class T>
T get_or(const ConfigLoader& cfg, const std::string& key, T fallback)
{
    try { return cfg.get<T>(key); } catch (...) { return fallback; }
}

void opt_float(const ConfigLoader& cfg, const std::string& key, std::optional<float>& dst)
{
    try { dst = static_cast<float>(cfg.get<double>(key)); } catch (...) {}
}

// body<-sensor mount from the DSR shadow.json, exactly as lidar3d_dds reads it (node by name, its
// incoming RT edge, Translation * RotX * RotY * RotZ, metres -> mm). The self-filter tests points in
// the ROBOT frame, so this transform IS the filter's correctness: there is no fallback. lidar3d_dds
// fell back to config numbers that had gone 45 mm stale once already (ROBOT_GEOMETRY.md); here a
// missing mount disables the filter LOUDLY instead of filtering against the wrong pose.
std::optional<Eigen::Affine3f> load_mount(const std::string& path, const std::string& node_name, std::string& why)
{
    QFile f(QString::fromStdString(path));
    if (not f.open(QIODevice::ReadOnly)) { why = "cannot open " + path; return std::nullopt; }
    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError)
    { why = "JSON parse error at byte " + std::to_string(err.offset) + ": " + err.errorString().toStdString(); return std::nullopt; }
    const QJsonObject symbols = doc.object().value("DSRModel").toObject().value("symbols").toObject();
    QString target;
    for (auto it = symbols.begin(); it != symbols.end(); ++it)
        if (it.value().toObject().value("name").toString() == QString::fromStdString(node_name)) { target = it.key(); break; }
    if (target.isEmpty()) { why = "no node named '" + node_name + "'"; return std::nullopt; }
    for (auto it = symbols.begin(); it != symbols.end(); ++it)
        for (const auto& lv : it.value().toObject().value("links").toArray())
        {
            const QJsonObject lk = lv.toObject();
            if (lk.value("label").toString() != "RT" or lk.value("dst").toString() != target) continue;
            const QJsonObject la = lk.value("linkAttribute").toObject();
            const QJsonArray r = la.value("rt_rotation_euler_xyz").toObject().value("value").toArray();
            const QJsonArray t = la.value("rt_translation").toObject().value("value").toArray();
            if (r.size() != 3 or t.size() != 3) { why = "RT edge arrays are not 3-element"; return std::nullopt; }
            Eigen::Affine3f a{Eigen::Translation3f(static_cast<float>(t[0].toDouble() * 1000.0),
                                                   static_cast<float>(t[1].toDouble() * 1000.0),
                                                   static_cast<float>(t[2].toDouble() * 1000.0))};
            a.rotate(Eigen::AngleAxisf(static_cast<float>(r[0].toDouble()), Eigen::Vector3f::UnitX())
                   * Eigen::AngleAxisf(static_cast<float>(r[1].toDouble()), Eigen::Vector3f::UnitY())
                   * Eigen::AngleAxisf(static_cast<float>(r[2].toDouble()), Eigen::Vector3f::UnitZ()));
            return a;
        }
    why = "nothing has an RT edge into '" + node_name + "'";
    return std::nullopt;
}

rc::media::PublisherConfig pub_config(const ConfigLoader& cfg, const std::string& topic,
                                      int depth_default, bool ds_default, const std::string& prefix)
{
    rc::media::PublisherConfig pc;
    pc.domain_id          = static_cast<std::uint32_t>(get_or<int>(cfg, "MediaPlane.Domain", 7));
    pc.topic_name         = topic;
    pc.history_depth      = get_or<int>(cfg, "MediaPlane." + prefix + "HistoryDepth", depth_default);
    pc.shared_memory_only = get_or<bool>(cfg, "MediaPlane.SharedMemoryOnly", true);
    pc.data_sharing       = get_or<bool>(cfg, "MediaPlane." + prefix + "DataSharing", ds_default);
    return pc;
}

rc::media::MediaDescriptor descriptor_base(const rc::media::PublisherConfig& pc, const char* type_name,
                                           const char* type_tag)
{
    rc::media::MediaDescriptor d;
    d.version            = 1;
    d.domain_id          = pc.domain_id;
    d.type_name          = type_name;
    d.type_tag           = type_tag;
    d.history_depth      = pc.history_depth;
    d.shared_memory_only = pc.shared_memory_only;
    d.data_sharing       = pc.data_sharing;
    d.ready              = true;
    return d;
}
}  // namespace

// ─────────────────────────────────── LidarStream ───────────────────────────────────

LidarStream::~LidarStream() { stop(); }

bool LidarStream::init(Config cfg, std::unique_ptr<MeshFilter> filter)
{
    cfg_ = std::move(cfg);
    filter_ = std::move(filter);
    ready_ = pub_.init(cfg_.pub);
    if (not ready_)
    {
        std::print(stderr, "[MediaPlane] {}: LiDAR writer init FAILED on '{}'\n", cfg_.name, cfg_.topic);
        return false;
    }
    std::print("[MediaPlane] {}: publishing '{}' (zero-copy {}, self-filter {})\n", cfg_.name, cfg_.topic,
               pub_.data_sharing_active() ? "yes" : "NO", filter_ ? "ON" : "OFF");
    worker_ = std::thread([this] { run(); });
    return true;
}

void LidarStream::submit(LidarScan&& scan)
{
    if (not ready_) return;
    {
        std::lock_guard lk(mtx_);
        if (pending_.has_value()) ++dropped_;   // the worker is still busy with an older one
        pending_ = std::move(scan);
    }
    cv_.notify_one();
}

void LidarStream::stop()
{
    {
        std::lock_guard lk(mtx_);
        stop_ = true;
    }
    cv_.notify_one();
    if (worker_.joinable()) worker_.join();
}

std::string LidarStream::descriptor() const
{
    std::lock_guard lk(desc_mtx_);
    return descriptor_;
}

void LidarStream::run()
{
    while (true)
    {
        LidarScan scan;
        {
            std::unique_lock lk(mtx_);
            cv_.wait(lk, [this] { return stop_ or pending_.has_value(); });
            if (stop_) return;
            scan = std::move(*pending_);
            pending_.reset();
        }
        process(scan);
    }
}

// The ring table and fan advertised with the plane, COMPUTED from the geometry that produced the
// points rather than clustered from them. Elevation convention is lidar3d_dds's: asin(z/r) in the
// device frame, + up, ascending — which for this reconstruction is exactly -elev_down per layer.
void LidarStream::set_geometry(const LidarScan& scan, const std::vector<char>& kept,
                               const std::vector<float>& elev_deg)
{
    auto m = cfg_.model;
    m.stamp_ms = wall_ms();
    m.ring_elev_deg.clear();
    for (std::size_t j = 0; j < kept.size(); ++j)
        if (kept[j]) m.ring_elev_deg.push_back(elev_deg[j]);
    std::ranges::sort(m.ring_elev_deg);
    m.rings = static_cast<int>(m.ring_elev_deg.size());
    if (not m.ring_elev_deg.empty())
    {
        m.fov_start_deg = m.ring_elev_deg.front();
        m.fov_end_deg   = m.ring_elev_deg.back();
    }
    m.azimuth_step_deg = static_cast<float>(scan.fov / scan.columns * 180.0 / M_PI);
    // Range/rate come from the Webots device; a [Lidar.<name>] key (already in cfg_.model) overrides.
    // Advertising only: nothing here filters points by these values.
    if (not m.range_max_m and scan.max_range_m > 0.0) m.range_max_m = static_cast<float>(scan.max_range_m);
    if (not m.range_min_m and scan.min_range_m >= 0.0) m.range_min_m = static_cast<float>(scan.min_range_m);
    if (not m.rate_hz and scan.rate_hz > 0.0)          m.rate_hz     = static_cast<float>(scan.rate_hz);

    auto d = descriptor_base(cfg_.pub, "LidarFrame", rc::media::LIDAR_FRAME_TYPE_TAG);
    d.streams["lidar"]      = cfg_.topic;
    d.stream_types["lidar"] = "LidarFrame";
    d.model = std::move(m);
    std::print("[MediaPlane] {}: {} of {} layers in the cloud, elevation [{:.2f} .. {:.2f}] deg, az step {:.4f} deg\n",
               cfg_.name, d.model.ring_elev_deg.size(), kept.size(),
               d.model.ring_elev_deg.empty() ? 0.f : d.model.ring_elev_deg.front(),
               d.model.ring_elev_deg.empty() ? 0.f : d.model.ring_elev_deg.back(), *d.model.azimuth_step_deg);
    std::lock_guard lk(desc_mtx_);
    descriptor_ = d.to_json();
}

void LidarStream::process(const LidarScan& scan)
{
    const int W = scan.columns, L = scan.layers;
    if (W <= 0 or L <= 0 or scan.range.size() != static_cast<std::size_t>(W) * L) return;

    // Beam directions, as the bridge has always reconstructed them (see the long note on the Webots
    // conventions in the old receiving_lidarData): row 0 is the TOP layer, the fan spans vfov over L-1
    // gaps, symmetric about the device horizon, tilt NOT applied. theta = pi + elev_down (pi = horizon).
    std::vector<float> hc(W), hs(W), vc(L), vs(L), elev_deg(L);
    std::vector<char> kept(L);
    for (int i = 0; i < W; ++i)
    {
        const double h = M_PI - i * (scan.fov / W) - scan.fov / 2.0;
        hc[i] = static_cast<float>(std::cos(h));
        hs[i] = static_cast<float>(std::sin(h));
    }
    const double v_step = L > 1 ? scan.vfov / (L - 1) : 0.0;
    for (int j = 0; j < L; ++j)
    {
        const double elev_down = -scan.vfov / 2.0 + j * v_step;
        const double v = M_PI + elev_down;
        vc[j] = static_cast<float>(std::cos(v));
        vs[j] = static_cast<float>(std::sin(v));
        elev_deg[j] = static_cast<float>(-elev_down * 180.0 / M_PI);
        // 1e-6 slack: the extreme channel lands a hair outside an edge set exactly on it.
        const bool in_window = elev_deg[j] >= cfg_.elev_min_deg - 1e-6 and elev_deg[j] <= cfg_.elev_max_deg + 1e-6;
        kept[j] = in_window and (not cfg_.up_half_only or v < M_PI);
    }
    if (not geometry_known_)
    {
        set_geometry(scan, kept, elev_deg);
        geometry_known_ = true;
    }

    // Points, horizontal-outer so the order is the one lidar3d_dds published. DEVICE frame in mm with
    // lidar3d_dds's axis swap (x' = -y, y' = x): the frame its consumers and the self-filter expect.
    RoboCompLidar3D::TData data;
    data.points.reserve(static_cast<std::size_t>(W) * L);
    for (int i = 0; i < W; ++i)
        for (int j = 0; j < L; ++j)
        {
            if (not kept[j]) continue;
            const float d = scan.range[static_cast<std::size_t>(j) * W + i];
            const float x = d * hc[i] * vc[j], y = d * hs[i] * vc[j], z = d * vs[j];
            if (not (std::isfinite(x) and std::isfinite(y) and std::isfinite(z))) continue;
            if (std::sqrt(x * x + y * y + z * z) <= 0.2f) continue;
            RoboCompLidar3D::TPoint p{};
            p.x = -y * 1000.f;
            p.y =  x * 1000.f;
            p.z =  z * 1000.f;
            data.points.push_back(p);
        }
    if (filter_)
        filter_->filter(data);

    const std::size_t n = data.points.size();
    if (n == 0 or n > rc::media::MAX_LIDAR_POINTS)
    {
        ++dropped_;
        return;
    }
    bool ok = false;
    if (rc::media::LidarFrame* s = pub_.loan(); s != nullptr)
    {
        s->stream_id(rc::media::STREAM_LIDAR);
        s->frame_id(frame_id_++);
        s->stamp_ms(static_cast<std::uint64_t>(scan.stamp_ms));
        s->count(static_cast<std::uint32_t>(n));
        s->stride(3);
        s->format(rc::media::LIDAR_FORMAT_XYZ_F32);
        float* out = s->points().data();
        for (std::size_t k = 0; k < n; ++k)   // mm -> m, straight into the loan
        {
            out[3 * k + 0] = data.points[k].x * 0.001f;
            out[3 * k + 1] = data.points[k].y * 0.001f;
            out[3 * k + 2] = data.points[k].z * 0.001f;
        }
        ok = pub_.publish(s);
    }
    ok ? ++published_ : ++dropped_;
}

// ─────────────────────────────────── MediaPlane ───────────────────────────────────

MediaPlane::MediaPlane() = default;
MediaPlane::~MediaPlane() { stop(); }

void MediaPlane::stop()
{
    if (helios_) helios_->stop();
    if (bpearl_) bpearl_->stop();
}

void MediaPlane::init(const ConfigLoader& cfg)
{
    // ── cameras + IMU ──
    zed_rgb_topic_   = get_or<std::string>(cfg, "MediaPlane.ZedRGBTopic",   "rc/zed/rgb");
    zed_depth_topic_ = get_or<std::string>(cfg, "MediaPlane.ZedDepthTopic", "rc/zed/depth");
    ricoh_topic_     = get_or<std::string>(cfg, "MediaPlane.RicohTopic",    "rc/ricoh/rgb");
    imu_topic_       = get_or<std::string>(cfg, "MediaPlane.ImuTopic",      "rc/imu/data");

    image_pub_cfg_ = pub_config(cfg, zed_rgb_topic_, 2, true, "");
    auto depth_cfg = image_pub_cfg_;  depth_cfg.topic_name = zed_depth_topic_;
    auto ricoh_cfg = image_pub_cfg_;  ricoh_cfg.topic_name = ricoh_topic_;
    // imu_dds ran history 8, data-sharing OFF: a 100-byte sample gains nothing from zero-copy.
    imu_pub_cfg_   = pub_config(cfg, imu_topic_, 8, false, "Imu");

    zed_ok_   = zed_rgb_.init(image_pub_cfg_) and zed_depth_.init(depth_cfg);
    ricoh_ok_ = ricoh_.init(ricoh_cfg);
    imu_ok_   = imu_.init(imu_pub_cfg_);
    std::print("[MediaPlane] zed {} | ricoh {} | imu {}  (domain {}, zero-copy zed={} ricoh={})\n",
               zed_ok_ ? "ok" : "FAILED", ricoh_ok_ ? "ok" : "FAILED", imu_ok_ ? "ok" : "FAILED",
               image_pub_cfg_.domain_id, zed_rgb_.data_sharing_active() and zed_depth_.data_sharing_active(),
               ricoh_.data_sharing_active());

    // ── LiDARs ──
    const std::string mount_file = get_or<std::string>(cfg, "MediaPlane.MountFile", "");
    const bool filter_on = get_or<bool>(cfg, "MeshFilter.enabled", true);
    const auto make_lidar = [&](const std::string& name, const std::string& default_topic, bool up_half_only)
    {
        const std::string k = "Lidar." + name + ".";
        LidarStream::Config lc;
        lc.name = name;
        lc.topic = get_or<std::string>(cfg, k + "Topic", default_topic);
        lc.pub = pub_config(cfg, lc.topic, 2, true, "");
        lc.elev_min_deg = get_or<double>(cfg, "LidarChannels." + name + ".ElevMinDeg", -1e9);
        lc.elev_max_deg = get_or<double>(cfg, "LidarChannels." + name + ".ElevMaxDeg", 1e9);
        lc.up_half_only = up_half_only;

        auto& m = lc.model;
        m.version = 1;
        m.source  = get_or<std::string>(cfg, k + "Source", "simulated");
        m.ref     = get_or<std::string>(cfg, k + "Ref", "");
        m.frame   = name;
        opt_float(cfg, k + "RateHz", m.rate_hz);
        opt_float(cfg, k + "RangeMaxM", m.range_max_m);
        opt_float(cfg, k + "RangeMinM", m.range_min_m);
        opt_float(cfg, k + "RangeSigmaFloorM", m.range_sigma_floor_m);
        opt_float(cfg, k + "RangeKRel", m.range_k_rel);
        opt_float(cfg, k + "RangeKIncidence", m.range_k_incidence);

        std::unique_ptr<MeshFilter> filter;
        if (filter_on)
        {
            std::string why;
            const auto mount = load_mount(mount_file, name, why);
            if (not mount)
                std::print(stderr, "[MediaPlane] {}: ★ SELF-FILTER DISABLED — mount not readable from '{}': {}\n",
                           name, mount_file, why);
            else
            {
                MeshFilter::Config mc;
                mc.mount = *mount;
                mc.robot_name      = get_or<std::string>(cfg, "MeshFilter.robot_name", "Shadow");
                mc.geometry_source = get_or<std::string>(cfg, "MeshFilter.geometry_source", "proto");
                mc.proto_file      = get_or<std::string>(cfg, "MeshFilter.proto_file", "");
                mc.proto_include_bounding_objects = get_or<bool>(cfg, "MeshFilter.proto_include_bounding_objects", false);
                mc.mesh_dir  = get_or<std::string>(cfg, "MeshFilter.mesh_dir", mc.mesh_dir);
                mc.mesh_file = get_or<std::string>(cfg, "MeshFilter.mesh_file", mc.mesh_file);
                mc.floor_z = static_cast<float>(get_or<double>(cfg, "MeshFilter.floor_z", -1e5));
                mc.top_z   = static_cast<float>(get_or<double>(cfg, "MeshFilter.top_z", 1e5));
                mc.dilate  = static_cast<float>(get_or<double>(cfg, "MeshFilter.dilate", 0.05));
                mc.footprint_radius = static_cast<float>(get_or<double>(cfg, k + "FootprintRadius", 0.0));
                mc.footprint_z_min  = static_cast<float>(get_or<double>(cfg, k + "FootprintZMin", 110.0));
                mc.footprint_z_max  = static_cast<float>(get_or<double>(cfg, k + "FootprintZMax", 1350.0));
                mc.skirt_radius = static_cast<float>(get_or<double>(cfg, k + "SkirtRadius", 0.0));
                mc.skirt_z_min  = static_cast<float>(get_or<double>(cfg, k + "SkirtZMin", 110.0));
                mc.skirt_z_max  = static_cast<float>(get_or<double>(cfg, k + "SkirtZMax", 350.0));
                filter = std::make_unique<MeshFilter>();
                if (not filter->init(mc))
                {
                    std::print(stderr, "[MediaPlane] {}: ★ SELF-FILTER DISABLED — scene build failed\n", name);
                    filter.reset();
                }
            }
        }
        auto s = std::make_unique<LidarStream>();
        if (not s->init(std::move(lc), std::move(filter)))
            s.reset();
        return s;
    };
    helios_ = make_lidar("helios", "rc/lidar3d/points", false);
    bpearl_ = make_lidar("bpearl", "rc/lidar3d/bpearl", true);
    initialized_.store(true, std::memory_order_release);
}

bool MediaPlane::publish_zed(webots::Camera* cam, webots::RangeFinder* ranger, std::int64_t stamp_ms)
{
    if (not zed_ok_ or cam == nullptr or ranger == nullptr) return false;
    const unsigned char* bgra = cam->getImage();
    const float* range = ranger->getRangeImage();
    if (bgra == nullptr or range == nullptr) return false;   // no sample yet
    const int W = cam->getWidth(), H = cam->getHeight();
    const int DW = ranger->getWidth(), DH = ranger->getHeight();
    const std::size_t rgb_bytes = static_cast<std::size_t>(W) * H * 3;
    const std::size_t depth_bytes = static_cast<std::size_t>(DW) * DH * sizeof(float);
    bool ok = true;

    if (rgb_bytes <= rc::media::MAX_IMAGE_BYTES)
    {
        if (rc::media::ImageFrame* s = zed_rgb_.loan(); s != nullptr)
        {
            s->stream_id(rc::media::STREAM_ZED_RGB);
            s->frame_id(zed_rgb_id_++);
            s->stamp_ms(static_cast<std::uint64_t>(stamp_ms));
            s->width(W); s->height(H); s->step(W * 3);
            s->format(rc::media::FORMAT_RGB8);
            s->size(static_cast<std::uint32_t>(rgb_bytes));
            // Webots hands back BGRA; convert straight into the loan (no intermediate image, no copy).
            const cv::Mat src(H, W, CV_8UC4, const_cast<unsigned char*>(bgra));
            cv::Mat dst(H, W, CV_8UC3, s->data().data());
            cv::cvtColor(src, dst, cv::COLOR_BGRA2RGB);
            ok = zed_rgb_.publish(s) and ok;
        }
        else ok = false;
    }
    if (depth_bytes <= rc::media::MAX_IMAGE_BYTES)
    {
        if (rc::media::ImageFrame* s = zed_depth_.loan(); s != nullptr)
        {
            s->stream_id(rc::media::STREAM_ZED_DEPTH);
            s->frame_id(zed_depth_id_++);
            s->stamp_ms(static_cast<std::uint64_t>(stamp_ms));
            s->width(DW); s->height(DH); s->step(DW * sizeof(float));
            s->format(rc::media::FORMAT_DEPTH_F32);   // metres, as the RangeFinder reports
            s->size(static_cast<std::uint32_t>(depth_bytes));
            std::memcpy(s->data().data(), range, depth_bytes);
            ok = zed_depth_.publish(s) and ok;
        }
        else ok = false;
    }
    return ok;
}

bool MediaPlane::publish_360(webots::Camera* cam1, webots::Camera* cam2, std::int64_t stamp_ms)
{
    if (not ricoh_ok_ or cam1 == nullptr or cam2 == nullptr) return false;
    const int w = cam1->getWidth(), h = cam1->getHeight();
    if (cam2->getWidth() != w or cam2->getHeight() != h) return false;
    const unsigned char* b1 = cam1->getImage();
    const unsigned char* b2 = cam2->getImage();
    if (b1 == nullptr or b2 == nullptr) return false;
    const std::size_t bytes = static_cast<std::size_t>(2 * w) * h * 3;
    if (bytes > rc::media::MAX_IMAGE360_BYTES) return false;

    rc::media::Image360Frame* s = ricoh_.loan();
    if (s == nullptr) return false;
    s->stream_id(rc::media::STREAM_RICOH_RGB);
    s->frame_id(ricoh_id_++);
    s->stamp_ms(static_cast<std::uint64_t>(stamp_ms));
    s->width(2 * w); s->height(h); s->step(2 * w * 3);
    s->format(rc::media::IMG360_FORMAT_RGB8);
    s->size(static_cast<std::uint32_t>(bytes));
    // The bridge's layout, converted straight into the loan: [cam2 tail | cam1 (centred) | cam2 head],
    // i.e. the [cam1 | cam2] concat rolled right by half a camera so cam1's optical axis is the centre.
    const int shift = w / 2;
    const cv::Mat c1(h, w, CV_8UC4, const_cast<unsigned char*>(b1));
    const cv::Mat c2(h, w, CV_8UC4, const_cast<unsigned char*>(b2));
    cv::Mat pano(h, 2 * w, CV_8UC3, s->data().data());
    cv::Mat d0 = pano(cv::Rect(0, 0, shift, h));
    cv::Mat d1 = pano(cv::Rect(shift, 0, w, h));
    cv::Mat d2 = pano(cv::Rect(shift + w, 0, w - shift, h));
    cv::cvtColor(c2(cv::Rect(w - shift, 0, shift, h)), d0, cv::COLOR_BGRA2RGB);
    cv::cvtColor(c1, d1, cv::COLOR_BGRA2RGB);
    cv::cvtColor(c2(cv::Rect(0, 0, w - shift, h)), d2, cv::COLOR_BGRA2RGB);
    return ricoh_.publish(s);
}

bool MediaPlane::publish_imu(const rc::media::ImuFrame& sample)
{
    if (not imu_ok_) return false;
    rc::media::ImuFrame* f = imu_.loan();
    if (f == nullptr) return false;
    *f = sample;
    f->stream_id(rc::media::STREAM_IMU);
    f->frame_id(imu_id_++);
    const bool ok = imu_.publish(f);
    if (ok) imu_last_wall_ms_.store(wall_ms(), std::memory_order_relaxed);
    return ok;
}

void MediaPlane::submit_lidar(const std::string& name, webots::Lidar* lidar, std::int64_t stamp_ms)
{
    LidarStream* s = name == "helios" ? helios_.get() : name == "bpearl" ? bpearl_.get() : nullptr;
    if (s == nullptr or lidar == nullptr) return;
    const float* range = lidar->getRangeImage();
    if (range == nullptr) return;
    LidarScan scan;
    scan.columns = lidar->getHorizontalResolution();
    scan.layers  = lidar->getNumberOfLayers();
    scan.fov     = lidar->getFov();
    scan.vfov    = lidar->getVerticalFov();
    scan.min_range_m = lidar->getMinRange();
    scan.max_range_m = lidar->getMaxRange();
    // Effective delivery rate = min(1/sampling period, device frequency): Webots re-delivers the stale scan
    // when the period is shorter than the device's own frequency, so the period alone over-states the rate.
    const int period_ms = lidar->getSamplingPeriod();
    const double dev_hz = lidar->getFrequency();
    scan.rate_hz = period_ms > 0 ? std::min(1000.0 / period_ms, dev_hz) : dev_hz;
    scan.stamp_ms = stamp_ms;
    scan.range.assign(range, range + static_cast<std::size_t>(scan.columns) * scan.layers);
    s->submit(std::move(scan));
}

std::string MediaPlane::descriptor(const std::string& stream) const
{
    if (not initialized_.load(std::memory_order_acquire)) return {};
    if (stream == "helios") return helios_ ? helios_->descriptor() : std::string{};
    if (stream == "bpearl") return bpearl_ ? bpearl_->descriptor() : std::string{};
    if (stream == "zed" and zed_ok_)
    {
        auto d = descriptor_base(image_pub_cfg_, "ImageFrame", rc::media::IMAGE_FRAME_TYPE_TAG);
        d.streams["rgb"]   = zed_rgb_topic_;
        d.streams["depth"] = zed_depth_topic_;
        return d.to_json();
    }
    if (stream == "ricoh" and ricoh_ok_)
    {
        auto d = descriptor_base(image_pub_cfg_, "Image360Frame", rc::media::IMAGE360_FRAME_TYPE_TAG);
        d.streams["rgb360"]      = ricoh_topic_;
        d.stream_types["rgb360"] = "Image360Frame";
        return d.to_json();
    }
    if (stream == "imu" and imu_ok_)
    {
        // Not live -> no descriptor: a descriptor is robot_concept's signal to STOP bridging the IMU.
        const std::int64_t last = imu_last_wall_ms_.load(std::memory_order_relaxed);
        if (last == 0 or wall_ms() - last > 2000) return {};
        auto d = descriptor_base(imu_pub_cfg_, "ImuFrame", rc::media::IMU_FRAME_TYPE_TAG);
        d.streams["imu"]      = imu_topic_;
        d.stream_types["imu"] = "ImuFrame";
        return d.to_json();
    }
    return {};
}

}  // namespace shadow_dds
