#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <stdio.h>

#include <animation_skin.hpp>

using namespace webots;

Robot* robot;
Skin* skin = NULL;

int main(int argc, char **argv) {

  robot = new Robot();

  AnimationSkin anim = AnimationSkin("skin");
  anim.test();

  return 0;
}

/*
WbuBvhMotion bvh_motion = NULL;
int skin_bone_count = 0;
bool loop = true;
int i, j;
int *index_skin_to_bvh;
int end_frame_index = 0;
int root_bone_index = -1;
std::string *joint_name_list;

void load_motion_data()
{
  char *skin_device_name = NULL;
  char *motion_file_path = NULL;
  int scale = 20;
  int c;
  char cadena[20];
  skin_device_name = (char *) malloc(50 * sizeof(char));
  strcpy(skin_device_name, "skin");
  strcat(skin_device_name, cadena);
  motion_file_path = "../../motions/Walk. ";
  scale = 20;

  std::cout << "FLAG!" << std::endl;

  if (skin_device_name == NULL || motion_file_path == NULL) {
    fprintf(stderr, "Missing required arguments -d and -f.\n");
    return;
  }

  
  skin = robot->getSkin("skin");

  // Open a BVH animation file.
  bvh_motion = wbu_bvh_read_file(motion_file_path);
  if (bvh_motion == NULL) {
    return;
  }

  // Get the number of bones in the Skin device
  skin_bone_count = skin->getBoneCount();
  if (skin_bone_count == 0) {
    printf("The Skin model has no bones to animate.\n");
    return;
  }

  // Get the number of joints and frames in the BVH file.
  const int bvh_joint_count = wbu_bvh_get_joint_count(bvh_motion);
  const int bvh_frame_count = wbu_bvh_get_frame_count(bvh_motion);

  // Get the bone names in the Skin device
  // TODO: Comprobar liberación de memoria de este array
  joint_name_list = new std::string[skin_bone_count];

  for (i = 0; i < skin_bone_count; ++i) {
    const std::string name = skin->getBoneName(i);;
    joint_name_list[i] = name;
    if (joint_name_list->compare("Hips") == 0)
      root_bone_index = i;
  }

  // Optional: Print the list of names in the Skin model.
  // printf("Human model joins:\n");
  // for (i = 0; i < skin_bone_count; ++i)
  //   printf("  Joint %d: %s\n", i, joint_name_list[i]);

  if (root_bone_index < 0)
    fprintf(stderr, "Root joint not found\n");

  // Find correspondencies between the Skin's bones and BVH's joint.
  // For example 'hip' could be bone 0 in Skin device, and joint 5 in BVH motion file
  index_skin_to_bvh = (int *)malloc(skin_bone_count * sizeof(int));
  for (i = 0; i < skin_bone_count; ++i) {
    index_skin_to_bvh[i] = -1;

    if (i == 24 || i == 25 || i == 26 || i == 15 || i == 16 || i == 17)
      continue;

    std::string skin_name = joint_name_list[i];
    for (j = 0; j < bvh_joint_count; ++j) {
      const char *bvh_name = wbu_bvh_get_joint_name(bvh_motion, j);
      if (skin_name.compare(bvh_name) == 0)
      {
        index_skin_to_bvh[i] = j;
      }
    }
  }

  // Pass absolute and relative joint T pose orientation to BVH utility library
  for (i = 0; i < skin_bone_count; ++i) {
    if (index_skin_to_bvh[i] < 0)
      continue;

    const double *global_t_pose = skin->getBoneOrientation(i, true);
    wbu_bvh_set_model_t_pose(bvh_motion, global_t_pose, index_skin_to_bvh[i], true);
    const double *local_t_pose = skin->getBoneOrientation(i, false);
    wbu_bvh_set_model_t_pose(bvh_motion, local_t_pose, index_skin_to_bvh[i], false);
  }

  // Set factor converting from BVH skeleton scale to Webots skeleton scale.
  // Only translation values are scaled by this factor.
  wbu_bvh_set_scale(bvh_motion, scale);

  double initial_root_position[3] = {0.0, 0.0, 0.0};
  double root_position_offset[3] = {0.0, 0.0, 0.0};
  const double *skin_root_position = skin->getBonePosition(root_bone_index, false);
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
    printf("Motion\n", i, index_skin_to_bvh[i]);
}


void motion_step()
{
    const int current_frame_index = wbu_bvh_get_frame_index(bvh_motion);
    for (i = 0; i < skin_bone_count; ++i) {
      if (index_skin_to_bvh[i] < 0)
        continue;

      // Get joint rotation for each joint.
      // Note that we need to pass the joint index according to BVH file.
      const double *orientation = wbu_bvh_get_joint_rotation(bvh_motion, index_skin_to_bvh[i]);
      skin->setBoneOrientation(i, orientation, false);
    }

    // printf("%d\n", current_frame_index);

    const int remaining_frames = end_frame_index - current_frame_index;
    if (remaining_frames <= 4) 
    {
      if (loop && root_bone_index >= 0) 
      {
        wbu_bvh_goto_frame(bvh_motion, end_frame_index - 1);
      }
      wbu_bvh_goto_frame(bvh_motion, 1);  // skip initial pose
    } 
    else 
    {
      int f = 4;
      while (f > 0) 
      {
        wbu_bvh_step(bvh_motion);
        --f;
      }
    }

  
}

// TODO: Pasar gestión de memoria a la libreria
void cleanup()
{
  

  // Cleanup
  delete joint_name_list;
  free(index_skin_to_bvh);
  wbu_bvh_cleanup(bvh_motion);
  delete robot;

  
}

void set_frame_pose(int frame)
{

  
  wbu_bvh_goto_frame(bvh_motion, frame);
    const int current_frame_index = wbu_bvh_get_frame_index(bvh_motion);
    printf("%d\n", current_frame_index);
    for (i = 0; i < skin_bone_count; ++i) {
      if (index_skin_to_bvh[i] < 0)
        continue;

      // Get joint rotation for each joint.
      // Note that we need to pass the joint index according to BVH file.
      const double *orientation = wbu_bvh_get_joint_rotation(bvh_motion, index_skin_to_bvh[i]);
      skin->setBoneOrientation(i, orientation, false);
    }
  
}

*/



