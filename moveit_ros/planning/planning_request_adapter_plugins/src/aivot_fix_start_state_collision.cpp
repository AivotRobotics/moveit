#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>
#include <ros/ros.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection_acl/collision_env_acl.h>

namespace default_planner_request_adapters
{
    static const std::string LOGNAME("AivotCollisionAdapter");

    class AivotFixStartStateCollision : public planning_request_adapter::PlanningRequestAdapter
    {
    public:
        static const std::string DT_PARAM_NAME;

        AivotFixStartStateCollision()
                : planning_request_adapter::PlanningRequestAdapter()
        {}

        void initialize(const ros::NodeHandle& nh) override {
            ROS_INFO_NAMED(LOGNAME, "Initializing '%s'", getDescription().c_str());

            if (!nh.getParam(DT_PARAM_NAME, max_dt_offset_)) {
                max_dt_offset_ = 0.5;
                ROS_INFO_STREAM_NAMED(LOGNAME, "Param '" << DT_PARAM_NAME << "' was not set. Using default value: " << max_dt_offset_);
            } else {
                ROS_INFO_STREAM_NAMED(LOGNAME, "Param '" << DT_PARAM_NAME << "' was set to " << max_dt_offset_);
            }
        }

        std::string getDescription() const override
        {
            return "(Aivot) Fix Start State In Collision";
        }

        bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                          std::vector<std::size_t>& added_path_index) const override
        {
            ROS_INFO_NAMED(LOGNAME, "Running '%s'", getDescription().c_str());

            // get the specified start state
            moveit::core::RobotState start_state = planning_scene->getCurrentState();
            moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

            bool invokeStuckPlanner = false;

            // check if start state is in collision
            collision_detection::CollisionRequest creq;
            creq.group_name = req.group_name;
            collision_detection::CollisionResult cres;
            planning_scene->checkCollision(creq, cres, start_state);
            if (cres.collision) {
                // Rerun in verbose mode
                collision_detection::CollisionRequest vcreq = creq;
                collision_detection::CollisionResult vcres;
                vcreq.verbose = true;
                planning_scene->checkCollision(vcreq, vcres, start_state);

                if (creq.group_name.empty()) {
                    ROS_INFO_NAMED(LOGNAME, "Start state appears to be in collision");
                } else {
                    ROS_INFO_STREAM_NAMED(LOGNAME, "Start state appears to be in collision with respect to group "
                            << creq.group_name);
                }

                invokeStuckPlanner = true;
            }

            if (!invokeStuckPlanner) {
                // check if start state satisfies constraints
                if (!planning_scene->isStateValid(start_state, req.path_constraints, req.group_name)) {
                    ROS_INFO("Path constraints not satisfied for start state...");
                    // Rerun in verbose mode
                    planning_scene->isStateValid(start_state, req.path_constraints, req.group_name, true);

                    invokeStuckPlanner = true;
                }
            }

            if (invokeStuckPlanner) {
                const collision_detection::CollisionEnvACL* pCollisionEnvAcl =
                    dynamic_cast<const collision_detection::CollisionEnvACL*>(planning_scene->getCollisionEnv().get());
                assert(pCollisionEnvAcl != nullptr && "Expected collision env of type CollisionEnvACL");

                std::vector<moveit::core::RobotState> prefixStates = pCollisionEnvAcl->getUnstuckPath(start_state);
                // if prefixStates is non-empty, first node is the start state and last node is the un-collided state
                if (prefixStates.size() > 1) {
                    // set updated start state to last prefix entry
                    start_state = prefixStates.back();
                    prefixStates.pop_back();

                    planning_interface::MotionPlanRequest req2 = req;
                    moveit::core::robotStateToRobotStateMsg(start_state, req2.start_state);
                    bool solved = planner(planning_scene, req2, res);
                    ROS_INFO_STREAM_NAMED(LOGNAME, "Planning with updated start state returned " <<
                        std::boolalpha << solved);

                    if (solved && !res.trajectory_->empty())
                    {
                        size_t initialWayPointCount = res.trajectory_->getWayPointCount();
                        // heuristically decide a duration offset for the trajectory (induced by the additional point added as a
                        // prefix to the computed trajectory)
                        res.trajectory_->setWayPointDurationFromPrevious(
                            0, std::min(max_dt_offset_, res.trajectory_->getAverageSegmentDuration()));
                        // TODO: there's an opportunity to optimize these vector operations
                        for (auto prefix_state : prefixStates) {
                            res.trajectory_->addPrefixWayPoint(prefix_state, 0.0);
                            // we add a prefix point, so we need to bump any previously added index positions
                            for (std::size_t &added_index : added_path_index)
                                added_index++;
                            added_path_index.push_back(0);
                        }
                        ROS_INFO_STREAM_NAMED(LOGNAME, "Planning with updated start state" <<
                            ", initialWaypoints: " << initialWayPointCount <<
                            ", finalWayPoints: " << res.trajectory_->getWayPointCount());
                    }
                    return solved;
                } else {
                    ROS_WARN_STREAM_NAMED(LOGNAME, "Callback returned " << prefixStates.size() <<
                        " prefix states. Passing the original planning request to the planner.");
                    return planner(planning_scene, req, res);
                }
            }
            else
            {
                if (creq.group_name.empty())
                    ROS_INFO_NAMED(LOGNAME, "Start state is valid");
                else
                    ROS_INFO_STREAM_NAMED(LOGNAME, "Start state is valid with respect to group " << creq.group_name);
                return planner(planning_scene, req, res);
            }
        }

    private:
        double max_dt_offset_;
    };

const std::string AivotFixStartStateCollision::DT_PARAM_NAME = "start_state_max_dt";
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AivotFixStartStateCollision,
                            planning_request_adapter::PlanningRequestAdapter);