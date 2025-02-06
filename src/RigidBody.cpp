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

    linearAcceleration = netForce*inverseMass + ownerWorld->getGravity() - linearVelocity*linearVelocityDamping;
    linearVelocity += linearAcceleration*delta;
    
    transform.translation += linearVelocity*delta;

    netForce = Vector3::zeros();
    netTorque = Vector3::zeros();
}

} // End of namespace UPhysics