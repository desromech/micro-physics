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
    }

    void computeMassDistribution()
    {
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

protected:
    float restitutionCoefficient = 0.2;

    Vector3 linearVelocity = Vector3::zeros();
    Vector3 linearAcceleration = Vector3::zeros();
    float linearVelocityDamping = 0.2;
    Vector3 netForce = Vector3::zeros();

    Vector3 angularVelocity = Vector3::zeros();
    Vector3 angularAcceleration = Vector3::zeros();
    float angularVelocityDamping = 0.2;
    Vector3 netTorque = Vector3::zeros();

    float mass = 0.0f;
    float inverseMass = 0.0f;
};
} // End of namespace UPhysics

#endif //UPHYSICS_RIGID_BODY_HPP

