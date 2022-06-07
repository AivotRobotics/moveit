#include <moveit/collision_detection_acl/collision_common.h>

namespace collision_detection
{
void CollisionData::enableGroup(const moveit::core::RobotModelConstPtr& robot_model)
{
  if (robot_model->hasJointModelGroup(req_->group_name))
    active_components_only_ = &robot_model->getJointModelGroup(req_->group_name)->getUpdatedLinkModelsSet();
  else
    active_components_only_ = nullptr;
}
}  // namespace collision_detection
