#ifndef UPHYSICS_COLLISION_SHAPE_HPP
#define UPHYSICS_COLLISION_SHAPE_HPP

#include "Vector3.hpp"
#include <vector>
#include <memory>

namespace UPhysics
{

typedef std::shared_ptr<class CollisionShape> CollisionShapePtr;

class CollisionShape
{
public:
    virtual bool isConvext() const
    {
        return false;
    }
};

class ConvexCollisionShape : public CollisionShape
{
public:
    virtual bool isConvext() const
    {
        return true;
    }

    virtual Vector3 computeLocalSupportInDirection(const Vector3 &D) const
    {
        return Vector3::zeros();
    }
};

class BoxCollisionShape : public ConvexCollisionShape
{
public:
    Vector3 halfExtent = Vector3(0.5f, 0.5f, 0.5f);
};

class SphereCollisionShape : public ConvexCollisionShape
{
public:
    float radius = 1.0f;
};

class ConvexHullCollisionShape : public CollisionShape
{
public:
    std::vector<Vector3> corners;
};

}; // End of namespace UPhysics

#endif //UPHYSICS_COLLISION_SHAPE_HPP
