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

    set_frame_pose(2);
}

void AnimationSkin::update(){
    //motion_step();
}

void AnimationSkin::load_motion_data(){

    // Open a BVH animation file.
    bvh_motion = read_motion_file(motion_file_path);
    if (bvh_motion->joint_list.empty()) {
        return;
    }

    // Get the number of joints and frames in the BVH file.
    const int bvh_joint_count = get_joint_count(bvh_motion);
    const int bvh_frame_count = get_frame_count(bvh_motion);

    /* std::cout << "-------------------------------------------" << std::endl;
    std::cout << "get_joint_count: " << get_joint_count(bvh_motion) << std::endl;
    std::cout << "get_frame_count: " << get_frame_count(bvh_motion) << std::endl;
    std::cout << "-------------------------------------------" << std::endl; */
 
    // Get the bone names in the Skin device
    std::string joint_name_list[skin_bone_count];

    for (int i = 0; i < skin_bone_count; ++i) {
        const std::string name = getBoneName(i);
        joint_name_list[i] = name;
        if (joint_name_list->compare("Hips") == 0)
            root_bone_index = i;
    }

    // Optional: Print the list of names in the Skin model.
    /* std::cout << "-------------------------------------------" << std::endl;
    std::cout << "Human model joints:" << std::endl;
    for (int i = 0; i < skin_bone_count; ++i)
      std::cout << "  Joint " << i << ": " << joint_name_list[i] << std::endl;
    std::cout << "-------------------------------------------" << std::endl; */


    if (root_bone_index < 0)
        std::cerr << "Root joint not found." << std::endl;

    // Find correspondencies between the Skin's bones and BVH's joint.
    // For example 'hip' could be bone 0 in Skin device, and joint 5 in BVH motion file
    for (int i = 0; i < skin_bone_count; ++i) {
        index_skin_to_bvh[i] = -1;

        if (i == 24 || i == 25 || i == 26 || i == 15 || i == 16 || i == 17)
          continue;

        std::string joint_name = joint_name_list[i];
        for (int j = 0; j < bvh_joint_count; ++j) {
            const std::string bvh_joint_name = get_joint_name(bvh_motion, j);
            if (joint_name.compare(bvh_joint_name) == 0)
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
    const int current_frame_index = bvh_motion->current_frame;

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

    printFrameInfo(2, 0);
    


    // TODO: Código duplicado
    for (int i = 0; i < skin_bone_count; ++i) {
        if (index_skin_to_bvh[i] < 0)
            continue;

        // Get joint rotation for each joint.
        // Note that we need to pass the joint index according to BVH file.

        const double *orientation = get_joint_rotation(bvh_motion, index_skin_to_bvh[i]);
        setBoneOrientation(i, orientation, false);
        if(i == 0){
          std::cout << "Frame " << bvh_motion->current_frame << ": orientation: " << orientation[0] << " " << orientation[1] << " " << orientation[2] << " " << orientation[3] << std::endl;
        }
    }
  
}

//***********************************
//        Utility functions          
//***********************************

void AnimationSkin::printFrameInfo(int frame, int boneFilter){
  if(!bvh_motion)
    return;

  std::cout << "Frame " << frame << ": orientation: " << bvh_motion->joint_list[boneFilter]->frame_rotation[frame] << std::endl;
}

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
  motion->n_joints = motion->n_joints + 1;
  motion->joint_list.push_back(new_joint);

  // initialize the children list
  new_joint->n_children = 0;
  new_joint->children = NULL;


  std::string line;
  int indexesJoints = 0;
  while (std::getline(file, line)) {
    char* token = strtok(const_cast<char*>(line.c_str()), DELIM);

    // opening bracket of joint
    if (strcmp(token, "{") == 0)
      indexesJoints++;

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
        
        BvhMotionJoint* new_child_joint = new BvhMotionJoint;
        new_child_joint->name = strdup(child_name); // strdup crea una copia del nombre

        new_joint->children[new_joint->n_children - 1] = new_child_joint;  

        add_new_joint(file, motion, child_name, new_joint, channels_count);
    }

    // EndPoint
    if (strcmp(token, "End") == 0) {
      token = strtok(NULL, DELIM);
      if (strcmp(token, "Site") == 0) {
        if (!std::getline(file, line))
          break;
        
        char* token = strtok(const_cast<char*>(line.c_str()), DELIM);
        indexesJoints++;
        
        // end site offset
        token = strtok(const_cast<char*>(line.c_str()), DELIM);
        if (strcmp(token, "OFFSET") == 0) {
          token = strtok(NULL, DELIM);
          new_joint->bone_vector[0] = atof(token);
          token = strtok(NULL, DELIM);
          new_joint->bone_vector[1] = atof(token);
          token = strtok(NULL, DELIM);
          new_joint->bone_vector[2] = atof(token);
        }

        if (!std::getline(file, line))
          break;
      }
    }

    // closing bracket of the joint
    if (strcmp(token, "}") == 0) {
      --indexesJoints;

      if(indexesJoints == 0){
        *channels_count += new_joint->n_channels;

        /* if(new_joint->parent != nullptr)
          std::cout << "New Joint added. Name: " << new_joint->name << " parent: " << new_joint->parent->name << std::endl;
        else
          std::cout << "New Joint added. Name: " << new_joint->name << " parent: null" << std::endl; */

        return new_joint;
      }else
      continue;
      
    }
  }

  std::cerr << "add_new_joint: Error reading motion file and creating bones." << std::endl;
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
        return nullptr;
    }

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: read_file(): could not open '" << filename << "' file." << std::endl;
        delete motion;
        return nullptr;
    }

    int channels_count = 0;
    std::string line;

    while (std::getline(file, line)) {

      char* token = strtok(const_cast<char*>(line.c_str()), DELIM);

      /* //DEBUG
      std::cout << token << std::endl; */

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
              break;
          }
      }
    }

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

  

  // init joints
  for (int i = 0; i < motion->n_joints; ++i) {
    BvhMotionJoint *joint = motion->joint_list[i];

    if (joint->n_position_channels > 0)
      joint->frame_position = new double*[n_frames];
    if ((joint->n_channels - joint->n_position_channels) > 0)
      joint->frame_rotation = new Eigen::Quaterniond[n_frames];
  }

  int frame_index = 0;
  std::string line;
  int joint_index = 0;
  while (std::getline(file, line) && frame_index < motion->n_frames) {

    joint_index = 0;
    int motion_index = 0;

    while (joint_index < motion->n_joints && motion_index < frame_channels_count) {
      BvhMotionJoint *joint = motion->joint_list[joint_index];
      assert(motion_index + joint->n_channels <= frame_channels_count);

      // initialize frame_position if needed
      if (joint->n_position_channels > 0) {
        joint->frame_position[frame_index] = new double[3];
        int i = 0;
        for (; i < 3; ++i)
          joint->frame_position[frame_index][i] = 0;
      }

      // initialize variables needed to compute rotation
      Eigen::Quaterniond frame_rotation = Eigen::Quaterniond::Identity();

      Eigen::Vector3d axes[3];
      axes[0] = Eigen::Vector3d::UnitX();
      axes[1] = Eigen::Vector3d::UnitY();
      axes[2] = Eigen::Vector3d::UnitZ();

      // if(joint_index == 1 && frame_index == 4)
      // {
      //   printf("X: %f Y: %f Z: %f", axes[0], axes[1], axes[2]);
      //   exit(0);
      // }

      for (int channel_index = 0; channel_index < joint->n_channels; ++channel_index) {

        if (motion_index == 0 && channel_index == 0){
          token = strtok(const_cast<char*>(line.c_str()), DELIM);
        }
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
          
          Eigen::Quaterniond q(Eigen::AngleAxisd(angle * M_PI / 180.0, axes[rotation_index]));
          
          for (int j=0; j < 3; ++j) {
            if (j != rotation_index)
              axes[j] = q * axes[j];
          }
          frame_rotation = q * frame_rotation;
          frame_rotation.normalize();
        }
      }

      joint->frame_rotation[frame_index] = frame_rotation;
      motion_index += joint->n_channels;
      ++joint_index;

    }

    ++frame_index;

  } 
}

void AnimationSkin::cleanupBvhMotion(BvhMotion* motion) {
    // Liberar memoria para cada joint
    for (int i = 0; i < motion->n_joints; ++i) {
        cleanupBvhMotionJoint(motion->joint_list[i]);
        delete motion->joint_list[i];
    }

    // Limpiar la lista de joints
    motion->joint_list.clear();

    // Liberar el objeto BvhMotion
    delete motion;
}

void AnimationSkin::cleanupBvhMotionJoint(BvhMotionJoint* joint) {
    // Liberar memoria para el array de rotations
    delete[] joint->frame_rotation;
    delete[] joint->frame_position;

    // Liberar memoria para children
    for (int i = 0; i < joint->n_children; ++i) {
        cleanupBvhMotionJoint(joint->children[i]);
        delete joint->children[i];
    }
    
    // Limpiar la lista de children
    delete[] joint->children;
}

int AnimationSkin::get_joint_count(BvhMotion *motion){
  if (motion != NULL)
    return motion->n_joints;

  std::cerr << "Error: get_joint_count(): BvhMotion argument is NULL." << std::endl;
  return -1;
}

const std::string AnimationSkin::get_joint_name(const BvhMotion* motion, int joint_id){
  if (motion != NULL) {
    if (joint_id < motion->n_joints)
      return motion->joint_list[joint_id]->name;

    std::cerr << "Error: get_joint_name(): 'joint_id' argument (" << joint_id << ") is greater than the number of joints (" << motion->n_joints << ")." << std::endl;
    return "";
  }

  std::cerr << "Error: get_joint_name(): BvhMotion argument is NULL." << std::endl;
  return "";
}

int AnimationSkin::get_frame_count(const BvhMotion* motion){
  if (motion != NULL)
    return motion->n_frames;

  std::cerr << "Error: get_frame_count(): BvhMotion argument is NULL." << std::endl;
  return -1;
}

bool AnimationSkin::step(BvhMotion* motion){
  if (motion != NULL) {
    if (motion->current_frame < motion->n_frames - 1)
      motion->current_frame = motion->current_frame + 1;
    else
      motion->current_frame = 0;
    return true;
  }

  std::cerr << "Error: step(): BvhMotion argument is NULL." << std::endl;
  return false;
}

bool AnimationSkin::goto_frame(BvhMotion *motion, int frame_number){
  if (motion != NULL) {
    if (frame_number < motion->n_frames) {
      motion->current_frame = frame_number;
      return true;
    }

    std::cerr << "Error: goto_frame(): frame_number argument is greater than the number of frames." << std::endl;
    return false;
  }

  std::cerr << "Error: goto_frame(): BvhMotion argument is NULL." << std::endl;
  return false;
}

bool AnimationSkin::reset(BvhMotion* motion){
  if (motion != NULL) {
    motion->current_frame = 0;
    return true;
  }

  std::cerr << "Error: reset(): BvhMotion argument is NULL." << std::endl;
  return false;
}

void AnimationSkin::set_model_t_pose(const BvhMotion* motion, const double *axisAngle, int joint_id, bool global){
  Eigen::Quaterniond rotationQuaternion = Eigen::Quaterniond(axisAngle);

  if (global) {
        motion->joint_list[joint_id]->wbt_global_t_pose = rotationQuaternion;
  } else {
        motion->joint_list[joint_id]->wbt_local_t_pose = rotationQuaternion;
  }
}

void AnimationSkin::set_scale(BvhMotion* motion, double scale){
  if (motion != NULL)
    motion->scale_factor = 1.0 / scale;
  else
    std::cerr << "Error: set_scale(): BvhMotion argument is NULL." << std::endl;
}

const double* AnimationSkin::get_root_translation(const BvhMotion* motion){
  static double result[3];
  int frameIndex = motion->current_frame;
  double* framePosition = motion->joint_list[0]->frame_position[frameIndex];
  
  for (int i = 0; i < 3; ++i) {
    result[i] = framePosition[i] * motion->scale_factor;
  }

  return result;
}

const double* AnimationSkin::get_joint_rotation(const BvhMotion* motion, int joint_id){
  static double result[4];

  if (joint_id >= motion->n_joints) {
    std::cerr << "Error: get_joint_rotation(): 'joint_id' argument (" << joint_id << ") is greater than the number of joints (" << motion->n_joints << ")." << std::endl;

    result[0] = 0;
    result[1] = 0;
    result[2] = 0;
    result[3] = 0;
    return result;
  }

  BvhMotionJoint *joint = motion->joint_list[joint_id];

  Eigen::Quaterniond frame_rotation = joint->frame_rotation[motion->current_frame];

    // if(motion->current_frame == 4 && joint_id == 1)
    // {
    //     printf("%f\n ", frame_rotation.w);
    //     printf("%f\n ", frame_rotation.x);
    //     printf("%f\n ", frame_rotation.y);
    //     printf("%f\n ", frame_rotation.z);
    // }

  frame_rotation.normalize();

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
    eigen_quaternion_to_axis_angle(joint->wbt_local_t_pose, result);
    return result;
  }

  // current frame bone orientation based on T pose
  frame_rotation = joint->bvh_t_pose.conjugate() * frame_rotation;

  // convert rotation axis to Webots bone T pose coordinate system
  eigen_quaternion_to_axis_angle(joint->wbt_local_t_pose, result);
  double angle = result[3];
  Eigen::Quaterniond axis(0.0, result[0], result[1], result[2]);
  Eigen::Quaterniond g = joint->wbt_global_t_pose;
  axis = axis * g;
  axis = g.conjugate() * axis;
  eigen_quaternion_to_axis_angle(axis, result);

  frame_rotation = eigen_quaternion_from_axis_angle(result[0], result[1], result[2], angle);
  frame_rotation = joint->wbt_local_t_pose, frame_rotation;
  eigen_quaternion_to_axis_angle(frame_rotation, result);

  return result;
}

void AnimationSkin::eigen_quaternion_to_axis_angle(const Eigen::Quaterniond& q, double* axis_angle) {
    // Obtener la matriz de rotación asociada al cuaternión
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

    // Calcular el ángulo y el eje de rotación a partir de la matriz de rotación
    Eigen::AngleAxisd angleAxis(rotationMatrix);
    double angle = angleAxis.angle();
    Eigen::Vector3d axis = angleAxis.axis();

    // Almacenar el resultado en el array axis_angle
    axis_angle[0] = axis[0];
    axis_angle[1] = axis[1];
    axis_angle[2] = axis[2];
    axis_angle[3] = angle;
}

Eigen::Quaterniond AnimationSkin::eigen_quaternion_from_axis_angle(double x, double y, double z, double angle) {
    // Normalizar el eje de rotación
    double length = std::sqrt(x * x + y * y + z * z);
    if (length > 0.0) {
        x /= length;
        y /= length;
        z /= length;
    }

    // Crear un cuaternión a partir del ángulo de ejes normalizado
    Eigen::AngleAxisd axisAngle(angle, Eigen::Vector3d(x, y, z));
    Eigen::Quaterniond quaternion(axisAngle);

    return quaternion;
}