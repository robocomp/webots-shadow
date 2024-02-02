#include "bvh_util.h"
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

class AnimationSkin : public webots::Skin {
public:

    AnimationSkin(const std::string &name, std::string motion_file_path);

    void update();


private:

    int skin_bone_count;
    int *index_skin_to_bvh;
    WbuBvhMotion bvh_motion;
    int root_bone_index;
    double root_position_offset[3] = {0.0, 0.0, 0.0};
    double initial_root_position[3] = {0.0, 0.0, 0.0};
    int scale = 20;
    int end_frame_index = 0;
    bool loop = false;
    
};

#endif // ANIMATIONSKIN_H