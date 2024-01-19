#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <stdio.h>
#include <AnimationSkin.hpp>

#define TIME_STEP 32

using namespace webots;

Robot* robot;
Skin* skin = NULL;

int main(int argc, char **argv) {

  robot = new Robot();

  AnimationSkin anim = AnimationSkin("skin", "../../motions/Walk.bvh");

    while(robot->step(TIME_STEP) != -1){
      
      anim.update();

      robot->step(TIME_STEP);
    }

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

*/



