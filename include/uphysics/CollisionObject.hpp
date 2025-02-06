#ifndef UPHYSICS_COLLISION_OBJECT_HPP
#define UPHYSICS_COLLISION_OBJECT_HPP

#include "uphysics/TRSTransform.hpp"
#include "uphysics/CollisionShape.hpp"
#include <memory>
namespace UPhysics
{

typedef std::shared_ptr<class CollisionObject> CollisionObjectPtr;
typedef std::shared_ptr<class CollisionShape> CollisionShapePtr;

class PhysicsWorld;

class CollisionObject
{
public:
    CollisionObject();
    virtual ~CollisionObject();

    virtual void integrateMovement(float delta)
    {
        // By default do nothing.
    }

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

    virtual float getRestitutionCoefficient() const
    {
        return 0.0;
    }

    virtual Vector3 computeVelocityAtRelativePoint(const Vector3 &relativePoint)
    {
        return Vector3::zeros();
    }

    virtual void applyLinearImpulse(Vector3 impulse)
    {
        (void)impulse;
    }

    virtual void applyImpulseAtRelativePosition(Vector3 impulse, Vector3 relativePosition)
    {
        (void)relativePosition;
        applyLinearImpulse(impulse);
    }

    virtual bool needsCollisionDetection() const
    {
        return false;
    }

    uint32_t getID()
    {
        return monotonicID;
    }

    const Vector3 &getPosition() const
    {
        return transform.translation;
    }

    void setPosition(const Vector3 &newPosition)
    {
        transform.translation = newPosition;
        transformChanged();
    }

    const TRSTransform &getTransform() const
    {
        return transform;
    }

    void setTransform(const TRSTransform &newTransform)
    {
        transform = newTransform;
        transformChanged();
    }

    virtual void transformChanged()
    {
    }

    virtual AABox3 getWorldBoundingBox() const
    {
        return collisionShape->localBoundingBoxWithMargin.transformedWith(transform);
    }

    CollisionShapePtr collisionShape;
    PhysicsWorld *ownerWorld = nullptr;

protected:
    TRSTransform transform;
    uint32_t monotonicID;

};
} // End of namespace UPhysics

#endif //UPHYSICS_COLLISION_OBJECT_HPP
