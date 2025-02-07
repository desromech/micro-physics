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

    linearAcceleration = netForce*inverseMass + ownerWorld->getGravity() + linearEngineAcceleration;
    linearVelocity += linearAcceleration*delta;
    linearVelocity *= pow(linearVelocityDamping, delta);

    transform.translation += linearVelocity*delta;

    netForce = Vector3::zeros();
    netTorque = Vector3::zeros();
    transformChanged();
}

} // End of namespace UPhysics