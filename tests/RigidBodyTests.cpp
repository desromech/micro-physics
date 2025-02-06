#include "uphysics/PhysicsWorld.hpp"
#include "uphysics/RigidBody.hpp"

#include <stdlib.h>
#include <stdio.h>

#define TestAssert(expr) \
if(!(expr)) { \
    fprintf(stderr, "%s.%d: Test assertion failure: %s\n", __FILE__, __LINE__, #expr); \
    abort(); \
}

using namespace UPhysics;

auto UpdateTimestep = 1.0f/120.0f;

void testEmptyWorld()
{
    auto physicsWorld = std::make_shared<PhysicsWorld> ();
    for(int i = 0; i < 100; ++i)
        physicsWorld->updateTimestep(UpdateTimestep);

}

void testFallingBox()
{
    auto physicsWorld = std::make_shared<PhysicsWorld> ();

    auto boxCollisionShape = std::make_shared<BoxCollisionShape> ();
    boxCollisionShape->halfExtent = 0.5;

    auto rigidBody = std::make_shared<RigidBody> ();
    rigidBody->collisionShape = boxCollisionShape;
    rigidBody->setMass(1.0);
    rigidBody->computeMassDistribution();
    physicsWorld->addCollisionObject(rigidBody);

    for(int i = 0; i < 100; ++i)
    {
        physicsWorld->updateTimestep(UpdateTimestep);
        //printf("%f %f %f\n", rigidBody->transform.translation.x, rigidBody->transform.translation.y, rigidBody->transform.translation.z);
    };

    TestAssert(rigidBody->getPosition().x == 0);
    TestAssert(rigidBody->getPosition().y < -2);
    TestAssert(rigidBody->getPosition().z == 0);
}

void testFallingBoxWithFloor()
{
    auto physicsWorld = std::make_shared<PhysicsWorld> ();

    // Create the floor
    auto floorCollisionShape = std::make_shared<BoxCollisionShape> ();
    floorCollisionShape->halfExtent = Vector3(5, 0.25, 5);
    auto floorCollisionObject = std::make_shared<CollisionObject> ();
    floorCollisionObject->collisionShape = floorCollisionShape;
    physicsWorld->addCollisionObject(floorCollisionObject);

    // Create the fallling box
    auto boxCollisionShape = std::make_shared<BoxCollisionShape> ();
    boxCollisionShape->halfExtent = 0.5;

    auto rigidBody = std::make_shared<RigidBody> ();
    rigidBody->collisionShape = boxCollisionShape;
    rigidBody->setMass(1.0);
    rigidBody->computeMassDistribution();
    rigidBody->setPosition(Vector3(0, 2, 0));
    physicsWorld->addCollisionObject(rigidBody);

    for(int i = 0; i < 100; ++i)
    {
        physicsWorld->updateTimestep(UpdateTimestep);
        printf("%f %f %f\n", rigidBody->getPosition().x, rigidBody->getPosition().y, rigidBody->getPosition().z);
    };

    TestAssert(rigidBody->getPosition().x == 0);
    TestAssert(rigidBody->getPosition().y >= 0);
    TestAssert(rigidBody->getPosition().z == 0);
}

int main()
{
    testEmptyWorld();
    testFallingBox();
    testFallingBoxWithFloor();
    return 0;
}