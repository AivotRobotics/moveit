#include <moveit/collision_detection_acl/collision_env_acl.h>
#include <moveit/collision_detection_acl/collision_detector_allocator_acl.h>
#include <moveit/collision_detection_acl/collision_common.h>

namespace collision_detection
{
static const std::string NAME = "ACL";
constexpr char LOGNAME[] = "collision_detection.acl";

CollisionEnvACL::CollisionEnvACL(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : CollisionEnv(model, padding, scale)
  , profiler_(true)
{


  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
}

CollisionEnvACL::CollisionEnvACL(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, double padding,
                                 double scale)
  : CollisionEnv(model, world, padding, scale)
  , profiler_(true)
{


  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvACL::~CollisionEnvACL()
{
  getWorld()->removeObserver(observer_handle_);
}

CollisionEnvACL::CollisionEnvACL(const CollisionEnvACL& other, const WorldPtr& world)
  : CollisionEnv(other, world)
  , profiler_(true)
{


  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
}

void CollisionEnvACL::setCollisionCallback(const collision_detection::CollisionCallbackFn& collCbkFn)
{
  ROS_DEBUG_NAMED(LOGNAME, "setting CollisionCallback in Env obj %x", this);
  collCbkFn_ = collCbkFn;
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "collCbkFn is now " << collCbkFn_);
}

void CollisionEnvACL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                         const moveit::core::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvACL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                         const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionEnvACL::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                               const moveit::core::RobotState& state,
                                               const AllowedCollisionMatrix* acm) const
{
  ROS_DEBUG_ONCE_NAMED(LOGNAME, "checkSelfCollision");

  if (collCbkFn_.empty()) {
    return;
  }
  moveit::tools::Profiler& prof = const_cast<moveit::tools::Profiler&>(profiler_);
  prof.begin("self_collision");

  collCbkFn_(req, res, state);

  prof.end("self_collision");
}

void CollisionEnvACL::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                          const moveit::core::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvACL:: checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                          const moveit::core::RobotState& state,
                                          const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, state, &acm);
}

void CollisionEnvACL::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                          const moveit::core::RobotState& /*state1*/,
                                          const moveit::core::RobotState& /*state2*/) const
{
  ROS_ERROR_NAMED(LOGNAME, "Continuous collision not implemented");
}

void CollisionEnvACL::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                          const moveit::core::RobotState& /*state1*/,
                                          const moveit::core::RobotState& /*state2*/,
                                          const AllowedCollisionMatrix& /*acm*/) const
{
  ROS_ERROR_NAMED(LOGNAME, "Not implemented");
}

void CollisionEnvACL::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                const moveit::core::RobotState& state,
                                                const AllowedCollisionMatrix* acm) const
{
  assert(!collCbkFn_.empty());
  ROS_DEBUG_ONCE_NAMED(LOGNAME, "checkRobotCollision");

  moveit::tools::Profiler& prof = const_cast<moveit::tools::Profiler&>(profiler_);
  prof.begin("robot_collision");

  collCbkFn_(req, res, state);

  prof.end("robot_collision");

  if (req.distance)
  {
    assert(false);
  }
}

void CollisionEnvACL::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                   const moveit::core::RobotState& state) const
{
  // TODO
}

void CollisionEnvACL::distanceRobot(const DistanceRequest& req, DistanceResult& res,
                                    const moveit::core::RobotState& state) const
{
  // TODO
}

void CollisionEnvACL::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);




  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionEnvACL::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  if (action == World::DESTROY)
  {



  }
  else
  {
    if (action & (World::DESTROY | World::REMOVE_SHAPE))
    {
    }




  }
}

const std::string& CollisionDetectorAllocatorACL::getName() const
{
  return NAME;
}
}  // namespace collision_detection
