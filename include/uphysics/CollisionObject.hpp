#ifndef UPHYSICS_COLLISION_OBJECT_HPP
#define UPHYSICS_COLLISION_OBJECT_HPP

#include "uphysics/RigidTransform.hpp"
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
        return 0.2;
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

    virtual void applyMovePerMass(Vector3 movement)
    {
        (void)movement;
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

    const Quaternion &getOrientation() const
    {
        return transform.rotation;
    }

    void setPosition(const Vector3 &newPosition)
    {
        assert(!newPosition.hasNaN());
        transform.translation = newPosition;
        transformChanged();
    }

    void setPositionAndOrientation(const Vector3 &newPosition, const Quaternion &newOrientation)
    {
        transform.rotation = newOrientation;
        transform.translation = newPosition;
        transformChanged();
    }

    const RigidTransform &getTransform() const
    {
        return transform;
    }

    void setTransform(const RigidTransform &newTransform)
    {
        transform = newTransform;
        assert(!transform.translation.hasNaN());
        transformChanged();
    }

    virtual void transformChanged()
    {
    }

    virtual AABox3 getWorldBoundingBox() const
    {
        return collisionShape->localBoundingBoxWithMargin.transformedWith(transform);
    }

    virtual Vector3 getLinearVelocity() const
    {
        return Vector3::zeros();
    }


    virtual Vector3 getAngularVelocity() const
    {
        return Vector3::zeros();
    }
    
    virtual float computeAngularInertiaForRelativeContactPoint(const Vector3 &relativePoint, const Vector3 &normal)
    {
        return 0.0f;
    }

    CollisionShapePtr collisionShape;
    PhysicsWorld *ownerWorld = nullptr;

protected:
    RigidTransform transform;
    uint32_t monotonicID;

};
} // End of namespace UPhysics

#endif //UPHYSICS_COLLISION_OBJECT_HPP
