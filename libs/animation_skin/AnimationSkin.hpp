#include <webots/Skin.hpp>
#include <webots/Robot.hpp>
#include <iostream>
#include <cstdio>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <cstdbool>
#include <fstream>
#include <Eigen/Geometry>

#ifndef ANIMATIONSKIN_H
#define ANIMATIONSKIN_H

#define MAX_LINE 4096
#define D2R (((double)M_PI) / 180.0)
const char DELIM[] = " :,\t\r\n";

class AnimationSkin : public webots::Skin {
public:

    AnimationSkin(const std::string &name, std::string motion_file_path);

    enum BvhChannelType{
        X_POSITION = 0,
        Y_POSITION = 1,
        Z_POSITION = 2,
        X_ROTATION = 3,
        Y_ROTATION = 4,
        Z_ROTATION = 5
    };

    struct BvhMotionJoint{
        std::string name;  // name of the joint

        struct BvhMotionJoint *parent;              // pointer to parent joint. NULL for root joint.
        int n_children;                             // number of children
        struct BvhMotionJoint **children;           // list of pointers to children joints. NULL if there are no children

        int n_channels;                             // number of channels. Typically 6 (3 rotation and 3 translation) for root joints and 3 (3 rotation) otherwise
        int n_position_channels;                    // number of translation channels. Typically 3 for root joints and 0 otherwise
        std::vector<BvhChannelType> channels;       // list of channels in order. We need to know in what order to apply rotations
        Eigen::Quaterniond *frame_rotation;         // list of rotations per frame. List size is [motion->n_frames]
        double **frame_position;                    // list of translations per frame. List size is [motion->n_frames]x[3]

        double offset[3];                           // offset from the parent bone
        double bone_vector[3];                      // vector relative to parent representing the "bone head" -> "bone tail" vector. In many conventions
                                                    // (including Webots) this vector matches the bone Y-axis

        Eigen::Quaterniond bvh_t_pose;                   // joint orientation relative to parent to set the BVH skeleton in T pose
        Eigen::Quaterniond wbt_global_t_pose;            // joint absolute orientation to set the Webots skeleton in T pose
        Eigen::Quaterniond wbt_local_t_pose;             // joint orientation relative to parent to set the Webots skeleton in T pose
    };

    struct BvhMotion{
        double frame_time;                          // approximate time per frame
        int n_frames;                               // number of frames in the BVH motion file
        int current_frame;                          // index of the current frame during animation
        int n_joints;                               // number of joints
        double scale_factor;                        // scale factor for translation. Typically set according to bone lengths of BVH skeleton vs. target skeleton.
        std::vector<BvhMotionJoint*> joint_list;     // list of joints
    };

    void update();


private:

    std::string skin_device_name;
    std::string motion_file_path;
    BvhMotion* bvh_motion;

    std::vector<int> index_skin_to_bvh;    

    int skin_bone_count = 0;
    int root_bone_index = -1;
    int end_frame_index = 0;

    bool loop = true;
    int scale = 20;

    // Funciones de lectura y manipulación BVH
    

    BvhMotionJoint* add_new_joint(std::ifstream &file, BvhMotion* motion, std::string this_name, BvhMotionJoint *parent, int *channels_count);
    BvhMotion* read_motion_file(const std::string filename);   

    bool step(BvhMotion* motion);
    bool goto_frame(BvhMotion* motion, int frame_number);
    bool reset(BvhMotion* motion);
    
    void load_motion_data();
    void motion_step();
    void set_frame_pose(int frame);
    void compute_bone_vectors(BvhMotion* motion);
    void read_motion(std::ifstream &file, BvhMotion* motion, int frame_channels_count);

    int get_joint_count(BvhMotion* motion);
    const std::string get_joint_name(const BvhMotion* motion, int joint_id);
    int get_frame_count(const BvhMotion* motion);
    void set_model_t_pose(const BvhMotion* motion, const double *axisAngle, int joint_id, bool global);
    void set_scale(BvhMotion* motion, double scale);
    const double *get_root_translation(const BvhMotion* motion);
    const double* get_joint_rotation(const BvhMotion* motion, int joint_id);
    void eigen_quaternion_to_axis_angle(const Eigen::Quaterniond& q, double* axis_angle);
    Eigen::Quaterniond eigen_quaternion_from_axis_angle(double x, double y, double z, double angle);

    void printFrameInfo(int frame, int boneFilter);

    void cleanupBvhMotion(BvhMotion* motion);
    void cleanupBvhMotionJoint(BvhMotionJoint* joint);
};

#endif // ANIMATIONSKIN_H