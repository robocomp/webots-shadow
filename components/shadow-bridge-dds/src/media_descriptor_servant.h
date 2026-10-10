#pragma once
// One MediaPlaneDDS endpoint per sensor node. robot_concept queries each driver's descriptor on the
// driver's own port (Proxies.MediaPlaneDDS..4) and relays it onto the matching DSR node, so this bridge
// answers on those SAME ports, one adapter each, all under the identity "mediaplanedds" the proxies
// name. robocompdsl generates one servant per interface; five copies of one interface need this.

#include <functional>
#include <string>
#include <utility>

#include <MediaPlaneDDS.h>

class MediaDescriptorServant : public RoboCompMediaPlaneDDS::MediaPlaneDDS
{
public:
    explicit MediaDescriptorServant(std::function<std::string()> get) : get_(std::move(get)) {}
    std::string getMediaDescriptor(const Ice::Current&) override { return get_(); }

private:
    std::function<std::string()> get_;
};
