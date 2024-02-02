#include "AnimationSkin.hpp"

AnimationSkin::AnimationSkin(const std::string &name, std::string motion_file_path) : Skin(name)
{
  bvh_motion = wbu_bvh_read_file(motion_file_path.c_str());

  int i, j;

  // Get the number of bones in the Skin device
  skin_bone_count = getBoneCount();
  if (skin_bone_count == 0) {
    printf("The Skin model has no bones to animate.\n");
  }

    // Get the number of joints and frames in the BVH file.
  const int bvh_joint_count = wbu_bvh_get_joint_count(bvh_motion);
  const int bvh_frame_count = wbu_bvh_get_frame_count(bvh_motion);
  printf("The BVH file \"%s\" has %d joints, and %d frames.\n", motion_file_path, bvh_joint_count, bvh_frame_count);

  // Get the bone names in the Skin device
  std::string* joint_name_list = new std::string[skin_bone_count];
  root_bone_index = -1;
  for (i = 0; i < skin_bone_count; ++i) {
    const std::string name = getBoneName(i);
    joint_name_list[i] = name;
    if (strcmp(name.c_str(), "Hips") == 0)
      root_bone_index = i;
  }

  // Find correspondencies between the Skin's bones and BVH's joint.
  // For example 'hip' could be bone 0 in Skin device, and joint 5 in BVH motion file
  index_skin_to_bvh = (int *)malloc(skin_bone_count * sizeof(int));
  for (i = 0; i < skin_bone_count; ++i) {
    index_skin_to_bvh[i] = -1;

    if (i == 24 || i == 25 || i == 26 || i == 15 || i == 16 || i == 17)
      continue;

    const char *skin_name = joint_name_list[i].c_str();
    for (j = 0; j < bvh_joint_count; ++j) {
      const char *bvh_name = wbu_bvh_get_joint_name(bvh_motion, j);
      if (strcmp(skin_name, bvh_name) == 0)
        index_skin_to_bvh[i] = j;
    }
  }

  // Pass absolute and relative joint T pose orientation to BVH utility library
  for (i = 0; i < skin_bone_count; ++i) {
    if (index_skin_to_bvh[i] < 0)
      continue;
    const double *global_t_pose = getBoneOrientation(i, true);
    wbu_bvh_set_model_t_pose(bvh_motion, global_t_pose, index_skin_to_bvh[i], true);
    const double *local_t_pose = getBoneOrientation(i, false);
    wbu_bvh_set_model_t_pose(bvh_motion, local_t_pose, index_skin_to_bvh[i], false);
  }

  // Set factor converting from BVH skeleton scale to Webots skeleton scale.
  // Only translation values are scaled by this factor.
  wbu_bvh_set_scale(bvh_motion, scale);

  const double *skin_root_position = getBonePosition(root_bone_index, false);
  if (root_bone_index >= 0) {
    const double *current_root_position = wbu_bvh_get_root_translation(bvh_motion);
    // Use initial Skin position as zero reference position
    for (i = 0; i < 3; ++i) {
      root_position_offset[i] = skin_root_position[i] - current_root_position[i];
      initial_root_position[i] = current_root_position[i];
    }
  }

  // Check end frame index
  if (end_frame_index > 0 && end_frame_index >= bvh_frame_count) {
    fprintf(stderr, "Invalid end frame index %d. This motion has %d frames.\n", end_frame_index, bvh_frame_count);
    end_frame_index = bvh_frame_count;
  } else
    end_frame_index = bvh_frame_count;

  wbu_bvh_goto_frame(bvh_motion, 2);
}

void AnimationSkin::update(){
    for (int i = 0; i < skin_bone_count; ++i) {
      if (index_skin_to_bvh[i] < 0)
        continue;

      // Get joint rotation for each joint.
      // Note that we need to pass the joint index according to BVH file.
      const double *orientation = wbu_bvh_get_joint_rotation(bvh_motion, index_skin_to_bvh[i]);
      setBoneOrientation(i, orientation, false);
    }

    // Offset the position by a desired value if needed.
    const double *root_position;
    if (root_bone_index >= 0) {
      root_position = wbu_bvh_get_root_translation(bvh_motion);
      double position[3];
      for (int i = 0; i < 3; ++i)
        position[i] = root_position[i] + root_position_offset[i];
      setBonePosition(root_bone_index, position, false);
    }

    // Fetch the next animation frame.
    // The simulation update rate is lower than the BVH frame rate, so 4 BVH motion frames are fetched.
    const int current_frame_index = wbu_bvh_get_frame_index(bvh_motion);
    const int remaining_frames = end_frame_index - current_frame_index;
    if (remaining_frames <= 4) {
      if (loop && root_bone_index >= 0) {
        // Save new global position offset
        // based on last frame and not on loaded frame (1 over 4)
        wbu_bvh_goto_frame(bvh_motion, end_frame_index - 1);
        root_position = wbu_bvh_get_root_translation(bvh_motion);
        for (int i = 0; i < 3; ++i)
          root_position_offset[i] += root_position[i] - initial_root_position[i];
      }
      wbu_bvh_goto_frame(bvh_motion, 1);  // skip initial pose
    } else {
      int f = 4;
      while (f > 0) {
        wbu_bvh_step(bvh_motion);
        --f;
      }
    }
}