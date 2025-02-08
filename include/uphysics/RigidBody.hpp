#ifndef UPHYSICS_RIGID_BODY_HPP
#define UPHYSICS_RIGID_BODY_HPP

#include "CollisionObject.hpp"

namespace UPhysics
{
class RigidBody : public CollisionObject
{
public:
    RigidBody();
    virtual ~RigidBody() override;

    virtual void integrateMovement(float delta);

    void setMass(float newMass)
    {
        mass = newMass;
        if(newMass == 0)
            inverseMass = 0;
        else
            inverseMass = 1.0f / mass;
    }

    virtual bool hasCollisionResponse() const
    {
        return inverseMass != 0;
    }

    void computeMassDistribution()
    {
        inertiaTensor = collisionShape->computeInertiaTensorWithMass(mass);
        inverseInertiaTensor = inertiaTensor.inverse();
    }

    virtual float getMass() const override
    {
        return mass;
    }

    virtual float getMassReciprocal() const override
    {
        return inverseMass;
    }

    virtual float getRestitutionCoefficient() const
    {
        return restitutionCoefficient;
    }

    virtual void transformChanged() override
    {
        CollisionObject::transformChanged();

        rotationMatrix = transform.rotation.asMatrix3x3();
        auto transposedRotationMatrix = rotationMatrix;
        worldInertiaTensor = rotationMatrix * inertiaTensor * transposedRotationMatrix;
	    worldInverseInertiaTensor = rotationMatrix * inverseInertiaTensor * transposedRotationMatrix;
    }

    virtual Vector3 computeVelocityAtRelativePoint(const Vector3 &relativePoint)
    {
        return linearVelocity;
    }

    virtual bool needsCollisionDetection() const
    {
        return inverseMass != 0.0f;
    }

    virtual void applyLinearImpulse(Vector3 impulse)
    {
        linearVelocity += impulse*inverseMass;
    }

    virtual void applyImpulseAtRelativePosition(Vector3 impulse, Vector3 relativePosition)
    {
        (void)relativePosition;
        applyLinearImpulse(impulse);
    }

    virtual void applyMovePerMass(Vector3 movement)
    {
        auto delta = movement * inverseMass;
        assert(!delta.hasNaN());
        setPosition(getPosition() + delta);
    }

    virtual Vector3 getLinearVelocity() const override
    {
        return linearVelocity;
    }


    virtual Vector3 getAngularVelocity() const override
    {
        return angularVelocity;
    }

    void setLinearEngineAcceleration(const Vector3 &newLinearEngineAcceleration)
    {
        linearEngineAcceleration = newLinearEngineAcceleration;
    }

    const Vector3 &getLinearEngineAcceleration()
    {
        return linearEngineAcceleration;
    }

    virtual float computeAngularInertiaForRelativeContactPoint(const Vector3 &relativePoint, const Vector3 &normal)
    {
        auto torquePerUnitImpulse = relativePoint.cross(normal);
        auto rotationPerUnitImpulse = worldInverseInertiaTensor * torquePerUnitImpulse;
        return rotationPerUnitImpulse.cross(relativePoint).dot(normal);
    }

protected:
    float restitutionCoefficient = 0.1;

    Vector3 linearEngineAcceleration = Vector3::zeros();
    Vector3 linearVelocity = Vector3::zeros();
    Vector3 linearAcceleration = Vector3::zeros();
    float linearVelocityDamping = 0.8;
    Vector3 netForce = Vector3::zeros();

    Vector3 angularVelocity = Vector3::zeros();
    Vector3 angularAcceleration = Vector3::zeros();
    float angularVelocityDamping = 0.8;
    Vector3 netTorque = Vector3::zeros();

    float mass = 0.0f;
    float inverseMass = 0.0f;

    Matrix3x3 rotationMatrix = Matrix3x3::zeros();
    Matrix3x3 inertiaTensor = Matrix3x3::zeros();
    Matrix3x3 inverseInertiaTensor = Matrix3x3::zeros();
    Matrix3x3 worldInertiaTensor = Matrix3x3::zeros();
    Matrix3x3 worldInverseInertiaTensor = Matrix3x3::zeros();
};
} // End of namespace UPhysics

#endif //UPHYSICS_RIGID_BODY_HPP

