/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   BVH file format utility class to be used with 'Skin' node to animate the mesh.
 *                It provides function to read the BVH file and adapt it to the 'Skin' model mesh.
 */
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "quaternion.h"
#include "vector3.h"

#ifndef WBU_BVH_UTIL_H
#define WBU_BVH_UTIL_H

#define MAX_LINE 4096
#define D2R (((double)M_PI) / 180.0)
const char DELIM[] = " :,\t\r\n";

#ifdef __cplusplus
extern "C" {
#endif

typedef enum BvhChannelType {
  X_POSITION = 0,
  Y_POSITION = 1,
  Z_POSITION = 2,
  X_ROTATION = 3,
  Y_ROTATION = 4,
  Z_ROTATION = 5
} BvhChannelType_t;

typedef struct BvhMotionJointPrivate {
  char *name;  // name of the joint

  struct BvhMotionJointPrivate *parent;     // pointer to parent joint. NULL for root joint.
  int n_children;                           // number of children
  struct BvhMotionJointPrivate **children;  // list of pointers to children joints. NULL if there are no children

  int
    n_channels;  // number of channels. Typically 6 (3 rotation and 3 translation) for root joints and 3 (3 rotation) otherwise
  int n_position_channels;        // number of translation channels. Typically 3 for root joints and 0 otherwise
  BvhChannelType_t *channels;     // list of channels in order. We need to know in what order to apply rotations
  WbuQuaternion *frame_rotation;  // list of rotations per frame. List size is [motion->n_frames]
  double **frame_position;        // list of translations per frame. List size is [motion->n_frames]x[3]

  double offset[3];       // offset from the parent bone
  double bone_vector[3];  // vector relative to parent representing the "bone head" -> "bone tail" vector. In many conventions
                          // (including Webots) this vector matches the bone Y-axis

  WbuQuaternion bvh_t_pose;         // joint orientation relative to parent to set the BVH skeleton in T pose
  WbuQuaternion wbt_global_t_pose;  // joint absolute orientation to set the Webots skeleton in T pose
  WbuQuaternion wbt_local_t_pose;   // joint orientation relative to parent to set the Webots skeleton in T pose
} BvhMotionJointPrivate_t;

typedef struct WbuBvhMotionPrivate {
  double frame_time;  // approximate time per frame
  int n_frames;       // number of frames in the BVH motion file
  int current_frame;  // index of the current frame during animation
  int n_joints;       // number of joints
  double
    scale_factor;  // scale factor for translation. Typically set according to bone lengths of BVH skeleton vs. target skeleton.
  BvhMotionJointPrivate_t **joint_list;  // list of joints
} WbuBvhMotionPrivate_t;


typedef struct WbuBvhMotionPrivate *WbuBvhMotion;

WbuBvhMotion wbu_bvh_read_file(const char *filename);
void wbu_bvh_cleanup(WbuBvhMotion motion);

const char *wbu_bvh_get_filename(WbuBvhMotion motion);
int wbu_bvh_get_joint_count(const WbuBvhMotion motion);
const char *wbu_bvh_get_joint_name(const WbuBvhMotion motion, int joint_id);

int wbu_bvh_get_frame_count(const WbuBvhMotion motion);
int wbu_bvh_get_frame_index(const WbuBvhMotion motion);
bool wbu_bvh_step(WbuBvhMotion motion);
bool wbu_bvh_goto_frame(WbuBvhMotion motion, int frame_number);
bool wbu_bvh_reset(WbuBvhMotion motion);

void wbu_bvh_set_scale(WbuBvhMotion motion, double scale);
const double *wbu_bvh_get_root_translation(const WbuBvhMotion motion);
const double *wbu_bvh_get_joint_rotation(const WbuBvhMotion motion, int joint_id);

void wbu_bvh_set_model_t_pose(const WbuBvhMotion motion, const double *axisAngle, int joint_id, bool global);

#ifdef __cplusplus
}
#endif

#endif  // WBU_BVH_UTIL_H
