#include <webots/Skin.hpp>
#include <iostream>

#ifndef ANIMATIONSKIN_H
#define ANIMATIONSKIN_H

class AnimationSkin : public webots::Skin {
public:

    AnimationSkin(const std::string &name);

    void test();
};

#endif // ANIMATIONSKIN_H