#include "uphysics/RigidBody.hpp"
#include "uphysics/PhysicsWorld.hpp"

namespace UPhysics
{
RigidBody::RigidBody()
{
}

RigidBody::~RigidBody()
{
}

void RigidBody::integrateMovement(float delta)
{
    if(!ownerWorld)
        return;


    // integrate linear movement
    linearAcceleration = netForce*inverseMass + ownerWorld->getGravity() + linearInternalAcceleration;
    auto integratedVelocity = linearVelocity + linearAcceleration*delta;
    integratedVelocity *= pow(linearVelocityDamping, delta);
    linearVelocityIntegrationDelta = integratedVelocity - linearVelocity;
    linearVelocity = integratedVelocity;

    // Integrate angular movement.
    angularAcceleration = inverseInertiaTensor *netTorque;
    angularVelocity += angularAcceleration*delta;
    angularVelocity *= pow(angularVelocityDamping, delta);

    // Integrate position
    auto translationDelta = linearVelocity*delta;
    assert(!translationDelta.hasNaN());
    auto integratedPosition = getPosition() + translationDelta;

    // Integrate orientation
    auto integratedOrientation = Quaternion(angularVelocity * (0.5*delta), 0.0f).exp() * getOrientation();
    integratedOrientation = integratedOrientation.normalized();

    setPositionAndOrientation(integratedPosition, integratedOrientation);

    netForce = Vector3::zeros();
    netTorque = Vector3::zeros();
}

} // End of namespace UPhysics