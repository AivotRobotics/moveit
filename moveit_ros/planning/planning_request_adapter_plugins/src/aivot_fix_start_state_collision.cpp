#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>
#include <ros/ros.h>

namespace default_planner_request_adapters
{
    static const std::string LOGNAME("AivotCollisionAdapter");

    class AivotFixStartStateCollision : public planning_request_adapter::PlanningRequestAdapter
    {
    public:
        AivotFixStartStateCollision()
                : planning_request_adapter::PlanningRequestAdapter()
        {}

        void initialize(const ros::NodeHandle& nh) override
        {
            ROS_INFO_NAMED(LOGNAME, "Initializing '%s'", getDescription().c_str());
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
            return planner(planning_scene, req, res);
        }

    private:
    };
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AivotFixStartStateCollision,
                            planning_request_adapter::PlanningRequestAdapter);