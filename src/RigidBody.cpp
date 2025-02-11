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

void RigidBody::checkTimeToSleep(float weight)
{
    if(isSleeping())
        return;
    
    auto movementAmount = computeMovementAmount();
    averageMovementAmount = mix(averageMovementAmount, movementAmount, weight);

    if(averageMovementAmount <= SleepingMovementThreshold)
        setSleepingStateFactors();
}

void RigidBody::setSleepingStateFactors()
{
    isAwake_ = false;
    angularVelocity = Vector3::zeros();
    linearVelocity = Vector3::zeros();
    linearVelocityIntegrationDelta = Vector3::zeros();
    netForce = Vector3::zeros();
    netTorque = Vector3::zeros();
}

void RigidBody::integrateMovement(float delta)
{
    if(!ownerWorld)
        return;
    assert(isAwake_);

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

void RigidBody::wakeUp()
{
    if(isAwake_) return;
    if(inverseMass == 0) return;

    isAwake_ = true;
    averageMovementAmount = 2.0f;
    if(ownerWorld)
        ownerWorld->addAwakeRigidBody(std::static_pointer_cast<RigidBody> (shared_from_this()));
    
}

} // End of namespace UPhysics
