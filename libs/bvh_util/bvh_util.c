/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bvh_util.h"


#define D2R (((double)M_PI) / 180.0)






static void read_motion(FILE *file, WbuBvhMotion motion, int frame_channels_count) {
  int n_frames = motion->n_frames;
  char *token;
  int joint_index;

  // init joints
  for (joint_index = 0; joint_index < motion->n_joints; ++joint_index) {
    BvhMotionJointPrivate_t *joint = motion->joint_list[joint_index];
    if (joint->n_position_channels > 0)
      joint->frame_position = malloc(n_frames * sizeof(double *));
    if ((joint->n_channels - joint->n_position_channels) > 0)
      joint->frame_rotation = malloc(n_frames * sizeof(WbuQuaternion));
  }

  int frame_index = 0;
  char line[MAX_LINE];
  while (fgets(line, MAX_LINE, file) && frame_index < motion->n_frames) {
    joint_index = 0;
    int motion_index = 0;

    while (joint_index < motion->n_joints && motion_index < frame_channels_count) {
      BvhMotionJointPrivate_t *joint = motion->joint_list[joint_index];
      assert(motion_index + joint->n_channels <= frame_channels_count);

      // initialize frame_position if needed
      if (joint->n_position_channels > 0) {
        joint->frame_position[frame_index] = malloc(3 * sizeof(double));
        int i = 0;
        for (; i < 3; ++i)
          joint->frame_position[frame_index][i] = 0;
      }

      // initialize variables needed to compute rotation
      WbuQuaternion frame_rotation = wbu_quaternion_zero();
      WbuQuaternion q;
      WbuVector3 axes[3];
      axes[0] = wbu_vector3(X_AXIS);
      axes[1] = wbu_vector3(Y_AXIS);
      axes[2] = wbu_vector3(Z_AXIS);

      // if(joint_index == 1 && frame_index == 4)
      // {
      //   printf("X: %f Y: %f Z: %f", axes[0], axes[1], axes[2]);
      //   exit(0);
      // }
      
      

      int channel_index = 0;
      for (channel_index = 0; channel_index < joint->n_channels; ++channel_index) {
        if (motion_index == 0 && channel_index == 0)
          token = strtok(line, DELIM);
        else
          token = strtok(NULL, DELIM);
        double motion_value = atof(token);
        BvhChannelType_t channel_type = joint->channels[channel_index];
        if (channel_type <= Z_POSITION)
          // store position
          joint->frame_position[frame_index][channel_type] = motion_value;

        else if (joint->channels[channel_index] <= Z_ROTATION && motion_value != 0.0) {
          // compute and store rotation
          double angle = motion_value * D2R;
          int rotation_index = joint->channels[channel_index] - X_ROTATION;
          q = wbu_quaternion_from_axis_angle(axes[rotation_index].x, axes[rotation_index].y, axes[rotation_index].z, angle);
          int j = 0;
          for (; j < 3; ++j) {
            if (j != rotation_index)
              axes[j] = wbu_vector3_rotate_by_quaternion(axes[j], q);
          }
          frame_rotation = wbu_quaternion_multiply(q, frame_rotation);
          frame_rotation = wbu_quaternion_normalize(frame_rotation);
        }
      }

      joint->frame_rotation[frame_index] = frame_rotation;
      motion_index += joint->n_channels;
      ++joint_index;
    }

    ++frame_index;
  }
}


//***********************************
//          API functions            
//***********************************

WbuBvhMotion wbu_bvh_read_file(const char *filename) {
  // initialize the motion structure
  WbuBvhMotion motion = malloc(sizeof(WbuBvhMotionPrivate_t));
  motion->n_joints = 0;
  motion->scale_factor = 1.0;

  if (!filename || !filename[0]) {
    fprintf(stderr, "Error: wbu_bvh_read_file() called with NULL or empty 'filename' argument.\n");
    free(motion);
    return NULL;
  }

  FILE *file = fopen(filename, "r");
  if (!file) {
    fprintf(stderr, "Error: wbu_bvh_read_file(): could not open '%s' file.\n", filename);
    free(motion);
    return NULL;
  }

  int channels_count = 0;
  char line[MAX_LINE];
  while (fgets(line, MAX_LINE, file)) {
    char *token;
    token = strtok(line, DELIM);
    // skeleton section
    if (strcmp(token, "HIERARCHY") == 0)
      continue;
    // skeleton structure
    if (strcmp(token, "ROOT") == 0) {
      token = strtok(NULL, DELIM);
      char name[50];
      strcpy(name, token);
      motion->joint_list = NULL;
      add_new_joint(file, motion, name, NULL, &channels_count);
      compute_bone_vectors(motion);
    }
    // motion section
    if (strcmp(token, "MOTION") == 0)
      continue;
    // frame Count
    if (strcmp(token, "Frames") == 0) {
      token = strtok(NULL, DELIM);
      motion->n_frames = atoi(token);
    }
    // frame time and motion section
    if (strcmp(token, "Frame") == 0) {
      token = strtok(NULL, DELIM);
      if (strcmp(token, "Time") == 0) {
        token = strtok(NULL, DELIM);
        motion->frame_time = atof(token);
        read_motion(file, motion, channels_count);
        motion->current_frame = 0;
        // compute_frame(motion, motion->current_frame);
        // compute_bvh_T_pose(motion);
        break;
      }
    }
  }

  if (ferror(file))
    fprintf(stderr, "Error: wbu_bvh_read_file(): file '%s' is possibly empty.\n", filename);

  fclose(file);
  return motion;
}

void wbu_bvh_cleanup(WbuBvhMotion motion) {
  int n_frames = motion->n_frames;
  int i, j;
  for (i = 0; i < motion->n_joints; ++i) {
    BvhMotionJointPrivate_t *joint = motion->joint_list[i];
    if (joint->n_position_channels > 0) {
      for (j = 0; j < n_frames; ++j)
        free(joint->frame_position[j]);
      free(joint->frame_position);
    }
    free(joint->children);
    free(joint->frame_rotation);
    free(joint->channels);
    free(joint->name);
    free(joint);
  }
  free(motion->joint_list);
  free(motion);
  motion = NULL;
}

int wbu_bvh_get_joint_count(WbuBvhMotion motion) {
  if (motion != NULL)
    return motion->n_joints;

  fprintf(stderr, "Error: wbu_bvh_get_joint_count(): WbuBvhMotion argument is NULL.\n");
  return -1;
}

const char *wbu_bvh_get_joint_name(const WbuBvhMotion motion, int joint_id) {
  if (motion != NULL) {
    if (joint_id < motion->n_joints)
      return motion->joint_list[joint_id]->name;

    fprintf(stderr, "Error: wbu_bvh_get_joint_name(): 'joint_id' argument (%d) is greater than the number of joints (%d).\n",
            joint_id, motion->n_joints);
    return "";
  }

  fprintf(stderr, "Error: wbu_bvh_get_joint_name(): WbuBvhMotion argument is NULL.\n");
  return "";
}

int wbu_bvh_get_frame_count(const WbuBvhMotion motion) {
  if (motion != NULL)
    return motion->n_frames;

  fprintf(stderr, "Error: wbu_bvh_get_frame_count(): WbuBvhMotion argument is NULL.\n");
  return -1;
}

int wbu_bvh_get_frame_index(const WbuBvhMotion motion) {
  if (motion != NULL)
    return motion->current_frame;

  fprintf(stderr, "Error: wbu_bvh_get_frame_index(): WbuBvhMotion argument is NULL.\n");
  return -1;
}

bool wbu_bvh_step(WbuBvhMotion motion) {
  if (motion != NULL) {
    if (motion->current_frame < motion->n_frames - 1)
      motion->current_frame = motion->current_frame + 1;
    else
      motion->current_frame = 0;
    return true;
  }

  fprintf(stderr, "Error: wbu_bvh_step(): WbuBvhMotion argument is NULL.\n");
  return false;
}

bool wbu_bvh_goto_frame(WbuBvhMotion motion, int frame_number) {
  if (motion != NULL) {
    if (frame_number < motion->n_frames) {
      motion->current_frame = frame_number;
      return true;
    }

    fprintf(stderr, "Error: wbu_bvh_goto_frame(): frame_number argument is greater than the number of frames.\n");
    return false;
  }

  fprintf(stderr, "Error: wbu_bvh_goto_frame(): WbuBvhMotion argument is NULL.\n");
  return false;
}

bool wbu_bvh_reset(WbuBvhMotion motion) {
  if (motion != NULL) {
    motion->current_frame = 0;
    return true;
  }

  fprintf(stderr, "Error: wbu_bvh_reset(): WbuBvhMotion argument is NULL.\n");
  return false;
}

void wbu_bvh_set_model_t_pose(const WbuBvhMotion motion, const double *axisAngle, int joint_id, bool global) {
  if (global)
    motion->joint_list[joint_id]->wbt_global_t_pose =
      wbu_quaternion_from_axis_angle(axisAngle[0], axisAngle[1], axisAngle[2], axisAngle[3]);
  else
    motion->joint_list[joint_id]->wbt_local_t_pose =
      wbu_quaternion_from_axis_angle(axisAngle[0], axisAngle[1], axisAngle[2], axisAngle[3]);
}

void wbu_bvh_set_scale(WbuBvhMotion motion, double scale) {
  if (motion != NULL)
    motion->scale_factor = 1.0 / scale;
  else
    fprintf(stderr, "Error: wbu_bvh_set_scale(): WbuBvhMotion argument is NULL.\n");
}

const double *wbu_bvh_get_root_translation(const WbuBvhMotion motion) {
  static double result[3];
  int frame_index = motion->current_frame;
  double *frame_position = motion->joint_list[0]->frame_position[frame_index];
  int i = 0;
  for (; i < 3; ++i)
    result[i] = frame_position[i] * motion->scale_factor;
  return result;
}

const double *wbu_bvh_get_joint_rotation(const WbuBvhMotion motion, int joint_id) {
  static double result[4];
  if (joint_id >= motion->n_joints) {
    fprintf(stderr,
            "Error: wbu_bvh_get_joint_rotation(): 'joint_id' argument (%d) is greater than the number of joints (%d).\n",
            joint_id, motion->n_joints);
    result[0] = 0;
    result[1] = 0;
    result[2] = 0;
    result[3] = 0;
    return result;
  }
  BvhMotionJointPrivate_t *joint = motion->joint_list[joint_id];

  WbuQuaternion frame_rotation = joint->frame_rotation[motion->current_frame];

    // if(motion->current_frame == 4 && joint_id == 1)
    // {
    //     printf("%f\n ", frame_rotation.w);
    //     printf("%f\n ", frame_rotation.x);
    //     printf("%f\n ", frame_rotation.y);
    //     printf("%f\n ", frame_rotation.z);
    // }

  frame_rotation = wbu_quaternion_normalize(frame_rotation);
    // if(motion->current_frame == 4 && joint_id == 1)
    // {
    //     printf("%f\n ", frame_rotation.w);
    //     printf("%f\n ", frame_rotation.x);
    //     printf("%f\n ", frame_rotation.y);
    //     printf("%f\n ", frame_rotation.z);
    // }
  // retrieve BVH model T pose from first frame
  if (motion->current_frame == 0) {
    joint->bvh_t_pose = frame_rotation;
    wbu_quaternion_to_axis_angle(joint->wbt_local_t_pose, result);
    return result;
  }

  // current frame bone orientation based on T pose
  frame_rotation = wbu_quaternion_multiply(wbu_quaternion_conjugate(joint->bvh_t_pose), frame_rotation);

  // convert rotation axis to Webots bone T pose coordinate system
  wbu_quaternion_to_axis_angle(frame_rotation, result);
  double angle = result[3];
  WbuQuaternion axis = wbu_quaternion(0.0, result[0], result[1], result[2]);
  WbuQuaternion g = joint->wbt_global_t_pose;
  axis = wbu_quaternion_multiply(axis, g);
  axis = wbu_quaternion_multiply(wbu_quaternion_conjugate(g), axis);
  wbu_quaternion_to_axis_angle(axis, result);

  // add converted frame rotation to Webots bone T pose rotation
  frame_rotation = wbu_quaternion_from_axis_angle(result[0], result[1], result[2], angle);
  frame_rotation = wbu_quaternion_multiply(joint->wbt_local_t_pose, frame_rotation);
  wbu_quaternion_to_axis_angle(frame_rotation, result);
  return result;
}