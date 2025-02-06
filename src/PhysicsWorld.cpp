#include "uphysics/PhysicsWorld.hpp"

namespace UPhysics
{

void PhysicsWorld::evaluateForceGeneratorWithDeltaTime(float deltaTimestep)
{
    
}
void PhysicsWorld::integrateMovementWithDeltaTime(float deltaTimestep)
{
    // Integrate
    for (auto &object : collisionObjects)
        object->integrateMovement(deltaTimestep);
    
}

void PhysicsWorld::detectAndResolveCollisionsWithDeltaTime(float deltaTimestep)
{
    
}

void PhysicsWorld::sendToSleepRestingObjects(float deltaTimestep)
{
    
}

void PhysicsWorld::updateTimestep(float deltaTimestep)
{
    evaluateForceGeneratorWithDeltaTime(deltaTimestep);
    integrateMovementWithDeltaTime(deltaTimestep);
    detectAndResolveCollisionsWithDeltaTime(deltaTimestep);
    sendToSleepRestingObjects(deltaTimestep);
    
}

} // End of namespace UPhysics