// ======================================================================
// \title  SubtopologyTopologyDefs.hpp
// \brief subtopology definitions header
//
// ======================================================================
#ifndef Rfm69_SubtopologyTopologyDefs_hpp
#define Rfm69_SubtopologyTopologyDefs_hpp

#include <Fw/Logger/Logger.hpp>
#include "Rfm69Config/Rfm69SubtopologyConfig.hpp"

namespace Rfm69 {
struct SubtopologyState {
    Rfm69Device device;
};

struct TopologyState {
    SubtopologyState rfm69;
};
}  // namespace Rfm69
#endif  // Rfm69_SubtopologyTopologyDefs_hpp
