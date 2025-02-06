#include "uphysics/PhysicsWorld.hpp"
#include <stdio.h>

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
    auto broadphaseResult = computeBroadphasePairCandidates();
    if (broadphaseResult.empty())
        return;

    printf("Broadphase size %zu\n", broadphaseResult.size());
    computeNarrowPhaseContactsFromBroadphasePairs(broadphaseResult);
    resolveContactManifoldsCollisionsAndConstraints(contactManifoldCache.manifolds);
}

void PhysicsWorld::computeNarrowPhaseContactsFromBroadphasePairs(const std::vector<std::pair<CollisionObjectPtr, CollisionObjectPtr> > &broadphasePairs)
{
    contactManifoldCache.beginEpoch();
    for(auto &pair : broadphasePairs)
        detectNarrowPhaseCollisionOf(pair.first, pair.second);

    contactManifoldCache.endEpoch();
}

void PhysicsWorld::detectNarrowPhaseCollisionOf(const CollisionObjectPtr &firstCollisionObject, const CollisionObjectPtr &secondCollisionObject)
{
    auto firstShape = firstCollisionObject->collisionShape;
    auto firstTransform = firstCollisionObject->getTransform();

    auto secondShape = firstCollisionObject->collisionShape;
    auto secondTransform = firstCollisionObject->getTransform();

    auto separatingAxisHint = contactManifoldCache.lastSeparatingAxisFor(firstCollisionObject, secondCollisionObject);

    auto contactPoints = firstShape->detectAndComputeCollisionContactPointsAt(firstTransform, secondShape, secondTransform, separatingAxisHint);

    contactManifoldCache.insertContactsForWith(contactPoints, firstCollisionObject, secondCollisionObject);
}

std::vector<std::pair<CollisionObjectPtr, CollisionObjectPtr>> PhysicsWorld::computeBroadphasePairCandidates()
{
    std::vector<std::pair<CollisionObjectPtr, CollisionObjectPtr>> pairs;
    for(size_t i = 0; i < collisionObjects.size(); ++i)
    {
        auto firstCollisionObject = collisionObjects[i];
        auto firstWorldBoundingBox = firstCollisionObject->getWorldBoundingBox();
        for(size_t j = i + 1; j < collisionObjects.size(); ++j)
        {
            auto secondCollisionObject = collisionObjects[j];
            auto secondWorldBoundingBox = secondCollisionObject->getWorldBoundingBox();
            if(!firstWorldBoundingBox.hasIntersectionWithBox(secondWorldBoundingBox))
                continue;

            pairs.push_back(std::pair(firstCollisionObject, secondCollisionObject));
        }
    }

    return pairs;
}

void PhysicsWorld::resolveContactManifoldsCollisionsAndConstraints(const std::vector<ContactManifoldPtr> &manifolds)
{
    if(manifolds.empty())
        return;
    
    printf("Manifolds to solve: %zu\n", manifolds.size());
    std::vector<ContactPoint> contactList;
    for(auto &manifold : manifolds)
    {
        if(manifold->hasCollisionResponse())
        {
            contactList.insert(contactList.end(), manifold->points.begin(), manifold->points.begin() + manifold->size);
        }

        size_t rounds = 2;
        for(size_t i = 0; i < rounds*contactList.size(); ++i )
        {
            solveCollisionContactResponseList(contactList);
            solveCollisionContactConstraintList(contactList);
        }
    }
}

void PhysicsWorld::solveCollisionContactResponseList(std::vector<ContactPoint> &contactList)
{

}

void PhysicsWorld::solveCollisionContactConstraintList(std::vector<ContactPoint> &contactList)
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