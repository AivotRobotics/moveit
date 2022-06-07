#pragma once

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/macros/class_forward.h>

namespace collision_detection
{
MOVEIT_STRUCT_FORWARD(CollisionGeometryData);

/** \brief Wrapper around world, link and attached objects' geometry data. */
struct CollisionGeometryData
{
  /** \brief Constructor for a robot link collision geometry object. */
  CollisionGeometryData(const moveit::core::LinkModel* link, int index)
    : type(BodyTypes::ROBOT_LINK), shape_index(index)
  {
    ptr.link = link;
  }

  /** \brief Constructor for a new collision geometry object which is attached to the robot. */
  CollisionGeometryData(const moveit::core::AttachedBody* ab, int index)
    : type(BodyTypes::ROBOT_ATTACHED), shape_index(index)
  {
    ptr.ab = ab;
  }

  /** \brief Constructor for a new world collision geometry. */
  CollisionGeometryData(const World::Object* obj, int index) : type(BodyTypes::WORLD_OBJECT), shape_index(index)
  {
    ptr.obj = obj;
  }

  /** \brief Returns the name which is saved in the member pointed to in \e ptr. */
  const std::string& getID() const
  {
    switch (type)
    {
      case BodyTypes::ROBOT_LINK:
        return ptr.link->getName();
      case BodyTypes::ROBOT_ATTACHED:
        return ptr.ab->getName();
      default:
        break;
    }
    return ptr.obj->id_;
  }

  /** \brief Returns a string of the corresponding \e type. */
  std::string getTypeString() const
  {
    switch (type)
    {
      case BodyTypes::ROBOT_LINK:
        return "Robot link";
      case BodyTypes::ROBOT_ATTACHED:
        return "Robot attached";
      default:
        break;
    }
    return "Object";
  }

  /** \brief Check if two CollisionGeometryData objects point to the same source object. */
  bool sameObject(const CollisionGeometryData& other) const
  {
    return type == other.type && ptr.raw == other.ptr.raw;
  }

  /** \brief Indicates the body type of the object. */
  BodyType type;

  /** \brief Multiple \e CollisionGeometryData objects construct a collision object. The collision object refers to an
   *  array of coordinate transformations at a certain start index. The index of the transformation of a child \e
   *  CollisionGeometryData object is then given by adding the parent collision object index and the \e shape_index of a
   *  geometry data object. */
  int shape_index;

  /** \brief Points to the type of body which contains the geometry. */
  union
  {
    const moveit::core::LinkModel* link;
    const moveit::core::AttachedBody* ab;
    const World::Object* obj;
    const void* raw;
  } ptr;
};

/** \brief Data structure which is passed to the collision callback function of the collision manager. */
struct CollisionData
{
  CollisionData() : req_(nullptr), active_components_only_(nullptr), res_(nullptr), acm_(nullptr), done_(false)
  {
  }

  CollisionData(const CollisionRequest* req, CollisionResult* res, const AllowedCollisionMatrix* acm)
    : req_(req), active_components_only_(nullptr), res_(res), acm_(acm), done_(false)
  {
  }

  ~CollisionData()
  {
  }

  /** \brief Compute \e active_components_only_ based on the joint group specified in \e req_ */
  void enableGroup(const moveit::core::RobotModelConstPtr& robot_model);

  /** \brief The collision request passed by the user */
  const CollisionRequest* req_;

  /** \brief  If the collision request includes a group name, this set contains the pointers to the link models that
   *  are considered for collision.
   *
   *  If the pointer is NULL, all collisions are considered. */
  const std::set<const moveit::core::LinkModel*>* active_components_only_;

  /** \brief The user-specified response location. */
  CollisionResult* res_;

  /** \brief The user-specified collision matrix (may be NULL). */
  const AllowedCollisionMatrix* acm_;

  /** \brief Flag indicating whether collision checking is complete. */
  bool done_;
};

/** \brief Data structure which is passed to the distance callback function of the collision manager. */
struct DistanceData
{
  DistanceData(const DistanceRequest* req, DistanceResult* res) : req(req), res(res), done(false)
  {
  }
  ~DistanceData()
  {
  }

  /** \brief Distance query request information. */
  const DistanceRequest* req;

  /** \brief Distance query results information. */
  DistanceResult* res;

  /** \brief Indicates if distance query is finished. */
  bool done;
};
}  // namespace collision_detection
