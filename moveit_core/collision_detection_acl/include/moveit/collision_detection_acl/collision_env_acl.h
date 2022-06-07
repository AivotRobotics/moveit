#pragma once

#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection_acl/collision_common.h>

#include <thor_moveit_config/LinkPosShiftArray.h>
#include "moveit/profiler/profiler.h"

namespace collision_detection
{
/** \brief ACL implementation of the CollisionEnv */
class CollisionEnvACL : public CollisionEnv
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

private:
  /** \brief Callback function executed for each change to the world environment */
  void notifyObjectChange(const ObjectConstPtr& obj, World::Action action);

  World::ObserverHandle observer_handle_;
  std::map<std::string, Eigen::Vector3d> linkPosShiftMap_;
  moveit::tools::Profiler profiler_;
};
}  // namespace collision_detection
