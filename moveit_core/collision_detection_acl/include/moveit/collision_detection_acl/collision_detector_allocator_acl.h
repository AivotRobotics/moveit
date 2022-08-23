#pragma once

#include <rosconsole/macros_generated.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection_acl/collision_env_acl.h>

namespace collision_detection
{

MOVEIT_CLASS_FORWARD(CollisionDetectorAllocatorACL);  // Defines CollisionDetectorAllocatorACLPtr, ConstPtr, WeakPtr... etc

/** \brief An allocator for ACL collision detectors */
class CollisionDetectorAllocatorACL
  : public CollisionDetectorAllocatorTemplate<CollisionEnvACL, CollisionDetectorAllocatorACL>
  , public collision_detection::acl::CollisionCallback
  , public std::enable_shared_from_this<collision_detection::acl::CollisionCallback>
{
public:
  CollisionEnvPtr allocateEnv(const WorldPtr& world, const moveit::core::RobotModelConstPtr& robot_model) const override;

  CollisionEnvPtr allocateEnv(const moveit::core::RobotModelConstPtr& robot_model) const override;

  const std::string& getName() const override;

  void setCollisionCallback(const collision_detection::acl::CollisionCallbackPtr& collisionCallback) {
      collisionCallback_ = collisionCallback;
  }

  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state) override {
    assert(collisionCallback_ && "No collision callback is set!");
    collisionCallback_->checkRobotCollision(req, res, state);
  }

  virtual void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state) override {
      assert(collisionCallback_ && "No collision callback is set!");
      collisionCallback_->checkSelfCollision(req, res, state);
  }

private:
    acl::CollisionCallbackPtr collisionCallback_;
};
}  // namespace collision_detection
