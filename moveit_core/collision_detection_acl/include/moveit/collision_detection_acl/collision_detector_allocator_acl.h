#pragma once

#include <rosconsole/macros_generated.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_acl/collision_env_acl.h>

namespace collision_detection
{
/** \brief An allocator for ACL collision detectors */
class CollisionDetectorAllocatorACL
  : public CollisionDetectorAllocatorTemplate<CollisionEnvACL, CollisionDetectorAllocatorACL>
{
public:
  const std::string& getName() const override;
};
}  // namespace collision_detection
