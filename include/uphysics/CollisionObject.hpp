#ifndef UPHYSICS_COLLISION_OBJECT_HPP
#define UPHYSICS_COLLISION_OBJECT_HPP

#include "uphysics/TRSTransform.hpp"
#include "uphysics/CollisionShape.hpp"
#include <memory>
namespace UPhysics
{

typedef std::shared_ptr<class CollisionObject> CollisionObjectPtr;

class CollisionObject
{
public:
    CollisionObject();

    virtual bool hasCollisionResponse() const
    {
        return false;
    }

    virtual float getMass() const
    {
        return 0.0;
    }

    virtual float getMassReciprocal() const
    {
        return 0.0;
    }

    virtual Vector3 computeVelocityAtRelativePoint(const Vector3 &relativePoint)
    {
        return Vector3::zeros();
    }

    uint32_t getID()
    {
        return monotonicID;
    }

    TRSTransform transform;
    CollisionShapePtr collisionShape;
private:
    uint32_t monotonicID;

};
} // End of namespace UPhysics

#endif //UPHYSICS_COLLISION_OBJECT_HPP
