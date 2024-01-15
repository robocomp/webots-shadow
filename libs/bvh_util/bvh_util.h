#ifndef BVH_UTIL_H
#define BVH_UTIL_H

#include <stdio.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "quaternion.h"
#include "vector3.h"

typedef struct WbuBvhMotionPrivate *WbuBvhMotion;

// Funciones de lectura y manipulación BVH
WbuBvhMotion wbu_bvh_read_file(const char *filename);
void wbu_bvh_cleanup(WbuBvhMotion motion);
int wbu_bvh_get_joint_count(WbuBvhMotion motion);
const char *wbu_bvh_get_joint_name(const WbuBvhMotion motion, int joint_id);
int wbu_bvh_get_frame_count(const WbuBvhMotion motion);
int wbu_bvh_get_frame_index(const WbuBvhMotion motion);
bool wbu_bvh_step(WbuBvhMotion motion);
bool wbu_bvh_goto_frame(WbuBvhMotion motion, int frame_number);
bool wbu_bvh_reset(WbuBvhMotion motion);
void wbu_bvh_set_model_t_pose(const WbuBvhMotion motion, const double *axisAngle, int joint_id, bool global);
void wbu_bvh_set_scale(WbuBvhMotion motion, double scale);
const double *wbu_bvh_get_root_translation(const WbuBvhMotion motion);
const double *wbu_bvh_get_joint_rotation(const WbuBvhMotion motion, int joint_id);

#endif  // BVH_UTIL_H
