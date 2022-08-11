#pragma once

#include <moveit/collision_detection/collision_env.h>

#include "moveit/profiler/profiler.h"
#include "moveit/collision_detection_fcl/collision_env_fcl.h"

namespace collision_detection
{
/** \brief ACL implementation of the CollisionEnv */
class CollisionEnvACL : public CollisionEnvFCL
{
public:
  CollisionEnvACL() = delete;

  CollisionEnvACL(const moveit::core::RobotModelConstPtr& model, double padding = 0.0, double scale = 1.0);

  CollisionEnvACL(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, double padding = 0.0,
                  double scale = 1.0);

  CollisionEnvACL(const CollisionEnvACL& other, const WorldPtr& world);

  ~CollisionEnvACL() override;

  void setCollisionCallback(const collision_detection::CollisionCallbackFn& collFn);

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                          const moveit::core::RobotState& state) const override;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                          const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                           const moveit::core::RobotState& state) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                           const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2, const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2) const override;

  void distanceSelf(const DistanceRequest& req, DistanceResult& res,
                    const moveit::core::RobotState& state) const override;

  void distanceRobot(const DistanceRequest& req, DistanceResult& res,
                     const moveit::core::RobotState& state) const override;

  void setWorld(const WorldPtr& world) override;

protected:

  /** \brief Bundles the different checkSelfCollision functions into a single function */
  void checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm) const;

  /** \brief Bundles the different checkRobotCollision functions into a single function */
  void checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                 const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm) const;
};
}  // namespace collision_detection
