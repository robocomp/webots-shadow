#include "AnimationSkin.hpp"

AnimationSkin::AnimationSkin(const std::string &name, std::string motion_file_path) : 
                                            Skin(name), 
                                            motion_file_path(motion_file_path),
                                            skin_device_name(name){

    // Getting and checking bone count.
    skin_bone_count = getBoneCount();
    if (skin_bone_count == 0) {
        std::cerr << "The Skin model has no bones to animate." << std::endl;
        return;
    }
    index_skin_to_bvh.resize(skin_bone_count);

    if (motion_file_path.empty()) {
        std::cerr << "Missing motion file path." << std::endl;
        return;
    }

    load_motion_data();
}

void AnimationSkin::load_motion_data(){

    // Open a BVH animation file.
    bvh_motion = read_motion_file(motion_file_path);
    if (bvh_motion.joint_list.empty()) {
        return;
    }

    // Get the number of joints and frames in the BVH file.
    const int bvh_joint_count = get_joint_count(bvh_motion);
    const int bvh_frame_count = get_frame_count(bvh_motion);

    // Get the bone names in the Skin device
    std::string joint_name_list[skin_bone_count];

    for (int i = 0; i < skin_bone_count; ++i) {
        const std::string name = getBoneName(i);
        joint_name_list[i] = name;
        if (joint_name_list->compare("Hips") == 0)
            root_bone_index = i;
    }

    // Optional: Print the list of names in the Skin model.
    std::cout << "Human model joints:" << std::endl;
    for (int i = 0; i < skin_bone_count; ++i)
      std::cout << "  Joint " << i << ": " << joint_name_list[i] << std::endl;

    if (root_bone_index < 0)
        std::cerr << "Root joint not found." << std::endl;

    // Find correspondencies between the Skin's bones and BVH's joint.
    // For example 'hip' could be bone 0 in Skin device, and joint 5 in BVH motion file
    for (int i = 0; i < skin_bone_count; ++i) {
        index_skin_to_bvh[i] = -1;

        if (i == 24 || i == 25 || i == 26 || i == 15 || i == 16 || i == 17)
        continue;

        std::string skin_name = joint_name_list[i];
        for (int j = 0; j < bvh_joint_count; ++j) {
            const std::string bvh_name = get_joint_name(bvh_motion, j);
            if (skin_name.compare(bvh_name) == 0)
                index_skin_to_bvh[i] = j;

        }
    }

    // Pass absolute and relative joint T pose orientation to BVH utility library
    for (int i = 0; i < skin_bone_count; ++i) {
        if (index_skin_to_bvh[i] < 0)
            continue;

        const double *global_t_pose = getBoneOrientation(i, true);
        set_model_t_pose(bvh_motion, global_t_pose, index_skin_to_bvh[i], true);
        const double *local_t_pose = getBoneOrientation(i, false);
        set_model_t_pose(bvh_motion, local_t_pose, index_skin_to_bvh[i], false);
    }

    // Set factor converting from BVH skeleton scale to Webots skeleton scale.
    // Only translation values are scaled by this factor.
    set_scale(bvh_motion, scale);

    double initial_root_position[3] = {0.0, 0.0, 0.0};
    double root_position_offset[3] = {0.0, 0.0, 0.0};
    const double *skin_root_position = getBonePosition(root_bone_index, false);
    if (root_bone_index >= 0) {
        const double *current_root_position = get_root_translation(bvh_motion);

        // Use initial Skin position as zero reference position
        for (int i = 0; i < 3; ++i) {
            root_position_offset[i] = skin_root_position[i] - current_root_position[i];
            initial_root_position[i] = current_root_position[i];
        }
    }

    // Check end frame index
    if (end_frame_index > 0 && end_frame_index >= bvh_frame_count) {
        std::cerr << "Invalid end frame index " << end_frame_index << ". This motion has " << bvh_frame_count << " frames." << std::endl;
        end_frame_index = bvh_frame_count;
    } else
        end_frame_index = bvh_frame_count;
}

void AnimationSkin::motion_step()
{
    const int current_frame_index = get_frame_index(bvh_motion);
    for (int i = 0; i < skin_bone_count; ++i) {
        if (index_skin_to_bvh[i] < 0)
            continue;

        // Get joint rotation for each joint.
        // Note that we need to pass the joint index according to BVH file.
        const double *orientation = get_joint_rotation(bvh_motion, index_skin_to_bvh[i]);
        setBoneOrientation(i, orientation, false);
    }

    const int remaining_frames = end_frame_index - current_frame_index;
    if (remaining_frames <= 4) 
    {
      if (loop && root_bone_index >= 0) 
      {
        goto_frame(bvh_motion, end_frame_index - 1);
      }
      goto_frame(bvh_motion, 1);  // skip initial pose
    } 
    else 
    {
      int f = 4;
      while (f > 0) 
      {
        step(bvh_motion);
        --f;
      }
    }

  
}

void AnimationSkin::set_frame_pose(int frame)
{

    goto_frame(bvh_motion, frame);

    // TODO: Código duplicado
    const int current_frame_index = get_frame_index(bvh_motion);
    for (int i = 0; i < skin_bone_count; ++i) {
        if (index_skin_to_bvh[i] < 0)
            continue;

        // Get joint rotation for each joint.
        // Note that we need to pass the joint index according to BVH file.
        const double *orientation = get_joint_rotation(bvh_motion, index_skin_to_bvh[i]);
        setBoneOrientation(i, orientation, false);
    }
  
}

void AnimationSkin::update(){
    
}

//***********************************
//        Utility functions          
//***********************************

AnimationSkin::BvhMotionJoint* AnimationSkin::add_new_joint(std::ifstream &file, BvhMotion* motion, std::string this_name, BvhMotionJoint *parent, int *channels_count) {
  // create and init new joint
  BvhMotionJoint* new_joint = new BvhMotionJoint;
  new_joint->name = this_name;
  new_joint->parent = parent;
  new_joint->bvh_t_pose = Eigen::Quaterniond::Identity();
  new_joint->wbt_global_t_pose = Eigen::Quaterniond::Identity();
  new_joint->wbt_local_t_pose = Eigen::Quaterniond::Identity();
  new_joint->n_channels = 0;
  new_joint->n_position_channels = 0;
  new_joint->frame_position = nullptr;
  new_joint->frame_rotation = nullptr;

  // update the motion structure
  motion.n_joints = motion.n_joints + 1;
  motion.joint_list.push_back(new_joint);

  // initialize the children list
  new_joint->n_children = 0;
  new_joint->children = NULL;


  char line[MAX_LINE];

    ////////////////////////////////////////////////////////////

  while (fgets(line, MAX_LINE, file)) {
    char *token = strtok(line, DELIM);

    // opening bracket of joint
    if (strcmp(token, "{") == 0)
      continue;

    // offset: note that offsets are relative to the parent bone,
    //         and are not in the absolute reference frame
    if (strcmp(token, "OFFSET") == 0) {
      token = strtok(NULL, DELIM);
      new_joint->offset[0] = atof(token);
      token = strtok(NULL, DELIM);
      new_joint->offset[1] = atof(token);
      token = strtok(NULL, DELIM);
      new_joint->offset[2] = atof(token);
    }

    // channels
    if (strcmp(token, "CHANNELS") == 0) {
      token = strtok(NULL, DELIM);
      int n_channels = atoi(token);
      new_joint->n_channels = n_channels;
      new_joint->channels.resize(n_channels);

      int i = 0;
      for (i = 0; i < n_channels; ++i) {
        token = strtok(NULL, DELIM);
        if (strcmp(token, "Xrotation") == 0)
          new_joint->channels[i] = X_ROTATION;
        else if (strcmp(token, "Yrotation") == 0)
          new_joint->channels[i] = Y_ROTATION;
        else if (strcmp(token, "Zrotation") == 0)
          new_joint->channels[i] = Z_ROTATION;
        else if (strcmp(token, "Xposition") == 0) {
          new_joint->channels[i] = X_POSITION;
          ++new_joint->n_position_channels;
        } else if (strcmp(token, "Yposition") == 0) {
          new_joint->channels[i] = Y_POSITION;
          ++new_joint->n_position_channels;
        } else if (strcmp(token, "Zposition") == 0) {
          new_joint->channels[i] = Z_POSITION;
          ++new_joint->n_position_channels;
        }
      }
    }

    // child joints
    if (strcmp(token, "JOINT") == 0) {
        token = strtok(NULL, DELIM);
        new_joint->n_children = new_joint->n_children + 1;
        new_joint->children = (BvhMotionJoint **)realloc(new_joint->children, (new_joint->n_children) * sizeof(BvhMotionJoint *));

        char child_name[50];
        strcpy(child_name, token);
        
        // Crear un nuevo objeto BvhMotionJoint y asignar su puntero al array de children
        new_joint->children[new_joint->n_children - 1] = new BvhMotionJoint;
        new_joint->children[new_joint->n_children - 1]->name = strdup(child_name);  // strdup crea una copia del nombre

        // Llamar a la función recursiva para procesar el nuevo objeto
        add_new_joint(file, motion, child_name, new_joint->children[new_joint->n_children - 1], channels_count);
    }

    // EndPoint
    if (strcmp(token, "End") == 0) {
      token = strtok(NULL, DELIM);
      if (strcmp(token, "Site") == 0) {
        if (fgets(line, MAX_LINE, file) == NULL)
          break;
        if (fgets(line, MAX_LINE, file) == NULL)
          break;

        // end site offset
        token = strtok(line, DELIM);
        if (strcmp(token, "OFFSET") == 0) {
          token = strtok(NULL, DELIM);
          new_joint->bone_vector[0] = atof(token);
          token = strtok(NULL, DELIM);
          new_joint->bone_vector[1] = atof(token);
          token = strtok(NULL, DELIM);
          new_joint->bone_vector[2] = atof(token);
        }
        if (fgets(line, MAX_LINE, file) == NULL)
          break;
      }
    }

    // closing bracket of the joint
    if (strcmp(token, "}") == 0) {
      *channels_count += new_joint->n_channels;
      return new_joint;
    }
  }

  fprintf(stderr, "Error: read_file() Error reading skeleton information. Check syntax of file.\n");
  return nullptr;
}

AnimationSkin::BvhMotion* AnimationSkin::read_motion_file(const std::string filename){

    // initialize the motion structure
    BvhMotion* motion = new BvhMotion;
    motion->n_joints = 0;
    motion->scale_factor = 1.0;

    if (filename.empty()) {
        std::cerr << "Error: read_motion_file() called with NULL or empty 'filename' argument." << std::endl;
        delete motion;
        return;
    }

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: read_file(): could not open '" << filename << "' file." << std::endl;
        delete motion;
        return;
    }

    int channels_count = 0;
    std::string line;

    while (std::getline(file, line)) {

        char* token = strtok(const_cast<char*>(line.c_str()), DELIM);

        // skeleton section
        if (strcmp(token, "HIERARCHY") == 0)
            continue;

        // skeleton structure
        if (strcmp(token, "ROOT") == 0) {
            token = strtok(NULL, DELIM);
            std::string name = token;
            motion->joint_list.clear();
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

    if (file.fail())
        std::cerr << "Error: read_file(): file '" << filename << "' is possibly empty." << std::endl;

    file.close();

    return motion;
}

void AnimationSkin::compute_bone_vectors(BvhMotion* motion){
    for (int i = 0; i < motion->n_joints; ++i) {
        BvhMotionJoint *joint = motion->joint_list[i];
        int n_children = joint->n_children;

        if (n_children == 1) {
            joint->bone_vector[0] = joint->children[0]->offset[0];
            joint->bone_vector[1] = joint->children[0]->offset[1];
            joint->bone_vector[2] = joint->children[0]->offset[2];

        } else if (n_children > 1) {
            for (int j = 0; j < n_children; ++j) {
                joint->bone_vector[0] += joint->children[j]->offset[0];
                joint->bone_vector[1] += joint->children[j]->offset[1];
                joint->bone_vector[2] += joint->children[j]->offset[2];
            }

            joint->bone_vector[0] = joint->bone_vector[0] / n_children;
            joint->bone_vector[1] = joint->bone_vector[1] / n_children;
            joint->bone_vector[2] = joint->bone_vector[2] / n_children;

        }  // else already set when parsing End Site
    }
}

void AnimationSkin::read_motion(std::ifstream &file, BvhMotion* motion, int frame_channels_count) {
  int n_frames = motion->n_frames;
  char *token;
  int joint_index;

  // init joints
  for (joint_index = 0; joint_index < motion->n_joints; ++joint_index) {
    BvhMotionJoint *joint = motion->joint_list[joint_index];
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
      BvhMotionJoint *joint = motion->joint_list[joint_index];
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
        BvhChannelType channel_type = joint->channels[channel_index];
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