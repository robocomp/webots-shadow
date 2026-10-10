# Webots2Robocomp – Interface Reference

This document describes all the interfaces that the **Webots2Robocomp** component exposes (implemented) and publishes (pushed to IceStorm topics), together with the units of every variable.

---

## Legend

| Symbol | Meaning |
|--------|---------|
| ✅ Implemented | Method is fully functional |
| ⚠️ Not implemented | Method exists but prints a warning and returns an empty struct |
| 📡 Published | Data pushed proactively to an IceStorm topic |

---

## Implemented interfaces (server side)

### 1. Camera360RGB

Provides a stitched 360° RGB panoramic image from two cameras mounted on the robot.

```
interface Camera360RGB
    TImage getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)  ✅
```

#### Struct `TImage`

| Field | Type | Unit / Notes |
|-------|------|--------------|
| `width` | int | pixels |
| `height` | int | pixels |
| `depth` | int | bytes per pixel (3 = RGB) |
| `focalx` | int | pixels (focal length X) |
| `focaly` | int | pixels (focal length Y) |
| `timestamp` | long | milliseconds (Unix epoch) |
| `period` | float | milliseconds (actual capture period) |
| `compressed` | bool | always `false` |
| `image` | byte[] | raw RGB bytes, row-major |
| `roi` | TRoi | region-of-interest descriptor (pixels) |

#### `getROI` parameters

| Parameter | Unit |
|-----------|------|
| `cx`, `cy` | pixels – centre of ROI |
| `sx`, `sy` | pixels – ROI size |
| `roiwidth`, `roiheight` | pixels – output resolution |

> **Note**: parameters are accepted but currently ignored; the full stitched frame is always returned.

---

### 2. CameraRGBDSimple

Provides RGB image, depth map, and point cloud from the ZED stereo camera.

```
interface CameraRGBDSimple
    TRGBD  getAll   (string camera)   ✅
    TDepth getDepth (string camera)   ✅  (returns data from the range-finder)
    TImage getImage (string camera)   ✅  (returns ZED RGB image)
    TPoints getPoints (string camera) ✅  (returns ZED point cloud)
```

#### Struct `TImage`

| Field | Type | Unit / Notes |
|-------|------|--------------|
| `width` | int | pixels |
| `height` | int | pixels |
| `depth` | int | bytes per pixel |
| `focalx` | int | pixels |
| `focaly` | int | pixels |
| `alivetime` | long | milliseconds (Unix epoch) |
| `period` | float | milliseconds |
| `compressed` | bool | always `false` |
| `image` | byte[] | raw RGB bytes, row-major |

#### Struct `TDepth`

| Field | Type | Unit / Notes |
|-------|------|--------------|
| `width` | int | pixels |
| `height` | int | pixels |
| `focalx` | int | pixels |
| `focaly` | int | pixels |
| `alivetime` | long | milliseconds (Unix epoch) |
| `period` | float | milliseconds |
| `depthFactor` | float | metres (max range of the sensor) |
| `compressed` | bool | always `false` |
| `depth` | byte[] | packed `float32` array, each value in **metres × 10** (i.e. decimetres) |

> **Important**: the raw depth bytes encode `float32` values scaled by ×10 relative to the Webots range-finder output.  
> `getDepth` (range-finder): depth stored as `value * 10`, effectively in **decimetres**.  
> `getImage` / `getPoints` (ZED): depth in **metres** (no scaling applied).

#### Struct `TPoints` / `Point3D`

| Field | Type | Unit |
|-------|------|------|
| `x`, `y`, `z` | float | metres |
| `alivetime` | long | milliseconds (Unix epoch) |
| `period` | float | milliseconds |

---

### 3. IMU

Provides inertial measurement data from the robot's accelerometer and gyroscope.

```
interface IMU
    Acceleration getAcceleration ()  ✅
    Gyroscope    getAngularVel   ()  ✅
    DataImu      getDataImu      ()  ⚠️
    Magnetic     getMagneticFields() ⚠️
    Orientation  getOrientation  ()  ✅
    void         resetImu        ()  ⚠️
```

#### Struct `Acceleration`

| Field | Type | Unit |
|-------|------|------|
| `XAcc`, `YAcc`, `ZAcc` | float | m/s² (raw Webots accelerometer) |
| `timestamp` | long | milliseconds (Unix epoch) |

#### Struct `Gyroscope`

| Field | Type | Unit |
|-------|------|------|
| `XGyr`, `YGyr`, `ZGyr` | float | rad/s (raw Webots gyro) |
| `timestamp` | long | milliseconds (Unix epoch) |

#### Struct `Orientation`

| Field | Type | Unit |
|-------|------|------|
| `Roll` | float | radians (ZYX Euler decomposition from Webots rotation matrix) |
| `Pitch` | float | radians |
| `Yaw` | float | radians |
| `timestamp` | long | milliseconds (Unix epoch) |

#### Struct `Magnetic` ⚠️

Not implemented. Returns zeroed struct.

---

### 4. Laser

2D laser scan derived from the first layer of the **Helios** Lidar3D sensor.

```
interface Laser
    TLaserData    getLaserData              ()          ✅
    TLaserData    getLaserAndBStateData     (out TBaseState bState)  ✅
    LaserConfData getLaserConfData          ()          ✅
```

#### Struct `TData` (element of `TLaserData`)

| Field | Type | Unit |
|-------|------|------|
| `angle` | float | radians (horizontal angle, range [−π, π]) |
| `dist` | float | metres (2D distance in the XY plane) |

#### Struct `LaserConfData`

| Field | Type | Unit |
|-------|------|------|
| `maxDegrees` | int | radians (stored as int; equals sensor FOV) |
| `maxRange` | int | metres |
| `minRange` | int | metres |
| `angleRes` | float | radians/beam (FOV / horizontal resolution) |

---

### 5. Lidar3D

Full 3D point cloud from the **Helios** (long-range) and **BPearl** (short-range) sensors.

```
interface Lidar3D
    TData          getLidarData                     (string name, float start, float len, int decimationDegreeFactor)  ✅
    TColorCloudData getColorCloudData               ()  ⚠️
    TDataImage      getLidarDataArrayProyectedInImage(string name)  ⚠️
    TDataCategory   getLidarDataByCategory           (TCategories categories, long timestamp)  ⚠️
    TData           getLidarDataProyectedInImage     (string name)  ⚠️
    TData           getLidarDataWithThreshold2d      (string name, float distance, int decimationDegreeFactor)  ⚠️
```

#### `getLidarData` – `name` parameter

| Value | Sensor |
|-------|--------|
| `"helios"` | Hesai Helios 32-beam (long range) |
| `"bpearl"` | Robosense BPearl (short range, wide FOV) |

#### Struct `TPoint`

| Field | Type | Unit |
|-------|------|------|
| `x`, `y`, `z` | float | **metres** |
| `r` | float | metres (radial distance from sensor) |
| `phi` | float | radians (horizontal angle) |
| `theta` | float | radians (vertical angle) |
| `distance2d` | float | metres (distance in the XY plane: `√(x²+y²)`) |
| `intensity` | int | dimensionless (0 for Webots, not provided by simulator) |
| `pixelX`, `pixelY` | int | pixels (only for projected variants, otherwise 0) |

#### Struct `TData`

| Field | Type | Unit |
|-------|------|------|
| `points` | TPoint[] | see above |
| `period` | float | milliseconds (actual capture period) |
| `timestamp` | long | milliseconds (Unix epoch) |

---

### 6. OmniRobot

Controls and queries the omnidirectional mobile base (mecanum wheels).

```
interface OmniRobot
    void setSpeedBase    (float advx, float advz, float rot)  ✅
    void stopBase        ()                                   ✅
    void getBaseState    (out TBaseState state)               ✅
    void getBasePose     (out int x, out int z, out float alpha) ⚠️
    void correctOdometer (int x, int z, float alpha)         ⚠️
    void resetOdometer   ()                                   ⚠️
    void setOdometer     (TBaseState state)                   ⚠️
    void setOdometerPose (int x, int z, float alpha)         ⚠️
```

#### `setSpeedBase` parameters

| Parameter | Unit | Notes |
|-----------|------|-------|
| `advx` | mm/s | lateral (side) velocity; converted to m/s internally |
| `advz` | mm/s | forward (advance) velocity; converted to m/s internally |
| `rot` | rad/s | rotational velocity |

#### Struct `TBaseState` (returned by `getBaseState`)

| Field | Type | Unit |
|-------|------|------|
| `x` | float | metres (Webots world frame X) |
| `z` | float | metres (Webots world frame Y mapped to Z) |
| `alpha` | float | radians (rotation angle around vertical axis) |
| `correctedX`, `correctedZ` | float | metres (odometry-corrected; same as raw in this implementation) |
| `correctedAlpha` | float | radians |
| `advVx`, `advVz` | float | m/s |
| `rotV` | float | rad/s |
| `isMoving` | bool | — |

> **Robot geometry constants** (defined in `specificworker.h`):  
> `WHEEL_RADIUS = 0.05 m`, `LX = 0.135 m`, `LY = 0.237 m`,  
> `LINEAR_MAX_SPEED = 1.5 m/s`, `ANGULAR_MAX_SPEED = 4.03 rad/s`

---

### 7. VisualElements

Tracks detected objects (humans) in the Webots scene.

```
interface VisualElements
    TObjects getVisualObjects (TObjects objects)  ✅
    void     setVisualObjects (TObjects objects)  ⚠️
```

#### Struct `TObject`

| Field | Type | Unit |
|-------|------|------|
| `id` | int | dimensionless (object/human identifier) |
| `x` | float | metres (Webots world frame X) |
| `y` | float | metres (Webots world frame Y) |
| `type` | int | dimensionless |
| `left`, `top`, `right`, `bot` | int | pixels (bounding box; not set in this implementation) |
| `score` | float | dimensionless [0–1] |
| `depth` | float | metres |
| `vx`, `vy`, `vz` | float | m/s (velocity; not set) |
| `vrx`, `vry`, `vrz` | float | rad/s (angular velocity; not set) |

#### Struct `TObjects`

| Field | Type | Unit |
|-------|------|------|
| `objects` | TObject[] | — |
| `timestampimage` | long | milliseconds (Unix epoch) |
| `timestampgenerated` | long | milliseconds (Unix epoch) |
| `period` | float | milliseconds |

---

### 8. Webots2Robocomp

Provides access to Webots-specific simulation features.

```
interface Webots2Robocomp
    ObjectPose getObjectPose   (string DEF)                         ✅
    void       resetWebots     ()                                   ⚠️
    void       setDoorAngle    (string DEF, float angle)            ✅
    void       setPathToHuman  (int humanId, TPath path)            ✅
```

#### `getObjectPose` return type `ObjectPose`

| Field | Sub-field | Type | Unit |
|-------|-----------|------|------|
| `position` | `x`, `y`, `z` | float | **millimetres** (Webots metres × 1000) |
| `orientation` | `x`, `y`, `z`, `w` | float | quaternion (dimensionless, unit quaternion) |

#### `setDoorAngle` parameter

| Parameter | Type | Unit | Range |
|-----------|------|------|-------|
| `DEF` | string | — | Webots DEF of the door's **HingeJoint** node (the one carrying `position`) |
| `angle` | float | radians | [−π, 0] — clamped automatically |

Writes the hinge's `position` field directly: a supervisor teleport, no dynamics. An unknown `DEF`,
or a node without a `position` field, logs an error and moves nothing.

#### `setPathToHuman` – `TPath` points

| Field | Type | Unit |
|-------|------|------|
| `x`, `y` | float | metres (Webots world frame, after coordinate-frame transform) |
| `radius` | float | metres |

---

## Published interfaces (IceStorm topics)

### 9. FullPoseEstimationPub 📡

Published every compute cycle (~30 Hz). Carries the full 6-DoF pose and velocity of the robot derived directly from Webots supervisor data.

```
topic  FullPoseEstimationPub
    void newFullPose(FullPoseEuler pose)
```

#### Struct `FullPoseEuler`

| Field | Type | Unit | Notes |
|-------|------|------|-------|
| `x` | float | metres | World frame X |
| `y` | float | metres | World frame Y (height, ~0 for ground robot) |
| `z` | float | metres | World frame Z |
| `rx` | float | radians | Roll — always 0 in current implementation |
| `ry` | float | radians | Pitch — always 0 in current implementation |
| `rz` | float | radians | Yaw (heading) derived from `atan2` of orientation matrix |
| `vx` | float | m/s | Lateral velocity in robot frame |
| `vy` | float | m/s | Forward velocity in robot frame |
| `vz` | float | m/s | Vertical velocity — always 0 |
| `vrx`, `vry` | float | rad/s | Angular velocity X/Y — always 0 |
| `vrz` | float | rad/s | Yaw rate (from Webots `getVelocity()[5]`) |
| `adv` | float | m/s | Forward velocity in robot frame (= `vx`) |
| `side` | float | m/s | Lateral velocity in robot frame (= `vy`) |
| `rot` | float | rad/s | Yaw rate (= `vrz`) |
| `ax`, `ay`, `az` | float | m/s² | Linear acceleration — not set (0) |
| `arx`, `ary`, `arz` | float | rad/s² | Angular acceleration — not set (0) |
| `confidence` | int | dimensionless | Not set (0) |
| `timestamp` | long | milliseconds (Unix epoch) |  |

---

## Subscribed interfaces (IceStorm topics)

### 10. JoystickAdapter (subscription)

The component **subscribes** to this topic and uses incoming data to drive the robot base.

```
topic  JoystickAdapter
    void sendData(TData data)
```

#### Struct `TData`

| Field | Type | Notes |
|-------|------|-------|
| `id` | string | device identifier |
| `axes` | AxisParams[] | list of axis readings |
| `buttons` | ButtonParams[] | list of button events (ignored) |

#### Struct `AxisParams`

| Field | Type | Unit | Expected `name` values |
|-------|------|------|------------------------|
| `name` | string | — | `"advance"`, `"side"`, `"rotate"` |
| `value` | float | mm/s (`advance`, `side`) / rad/s (`rotate`) | passed directly to `setSpeedBase` |

---

## Summary table

| Interface | Role | Status |
|-----------|------|--------|
| Camera360RGB | server | ✅ fully implemented |
| CameraRGBDSimple | server | ✅ (`getDepth` / `getAll` from range-finder; `getImage` / `getPoints` from ZED) |
| IMU | server | ✅ acceleration, angular velocity, orientation; ⚠️ magnetic, full DataImu |
| Laser | server | ✅ (derived from Helios Lidar3D layer 0) |
| Lidar3D | server | ✅ `getLidarData`; ⚠️ other variants |
| OmniRobot | server | ✅ `setSpeedBase`, `stopBase`, `getBaseState`; ⚠️ odometry methods |
| VisualElements | server | ✅ `getVisualObjects`; ⚠️ `setVisualObjects` |
| Webots2Robocomp | server | ✅ `getObjectPose`, `setDoorAngle`, `setPathToHuman`; ⚠️ `resetWebots` |
| FullPoseEstimationPub | published topic | 📡 every compute cycle |
| JoystickAdapter | subscribed topic | drives OmniRobot base |

