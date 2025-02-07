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

    //printf("Broadphase size %zu\n", broadphaseResult.size());
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

    auto secondShape = secondCollisionObject->collisionShape;
    auto secondTransform = secondCollisionObject->getTransform();

    auto separatingAxisHint = contactManifoldCache.lastSeparatingAxisFor(firstCollisionObject, secondCollisionObject).safeNormalized();
    if(separatingAxisHint.closeTo(Vector3(0,0,0)))
        separatingAxisHint = Vector3(1, 0, 0);

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
    
    //printf("Manifolds to solve: %zu\n", manifolds.size());
    std::vector<ContactPoint> contactList;
    for(auto &manifold : manifolds)
    {
        if(manifold->hasCollisionResponse())
        {
            contactList.insert(contactList.end(), manifold->points.begin(), manifold->points.begin() + manifold->size);
        }
    }

    const size_t RoundCount = 1;
    //printf("Contact list size %zu\n", contactList.size());
    for(size_t rounds = 0; rounds < RoundCount*contactList.size(); ++rounds)
    {
        solveCollisionContactResponseList(contactList);
        solveCollisionContactConstraintList(contactList);
    }

}


void PhysicsWorld::solveCollisionContactResponseList(std::vector<ContactPoint> &contactList)
{
    if(contactList.empty())
        return;

    ContactPoint *bestFound = nullptr;
    float bestFoundClosingSpeed = -INFINITY;

    for(auto &contact : contactList)
    {
        if(contact.hasCollisionResponse() && contact.inverseInertia() > 0.0)
        {
            auto closingSpeed = contact.closingSpeed();
            if (!bestFound || closingSpeed > bestFoundClosingSpeed)
            {
                bestFound = &contact;
                bestFoundClosingSpeed = closingSpeed;
            }
        }
    }

    if(bestFound)
        solveCollisionContactResponse(*bestFound);
}

void PhysicsWorld::solveCollisionContactResponse(ContactPoint &contact)
{
    // Are they already separating?
    auto separatingSpeed = contact.separationSpeed();
    //printf("separationSpeed %f\n", separatingSpeed);
    if(separatingSpeed > 0)
        return;

    auto restitution = std::min(contact.firstCollisionObject->getRestitutionCoefficient(), contact.secondCollisionObject->getRestitutionCoefficient());
    
    auto newSeparatingSpeed = -separatingSpeed*restitution;
    auto deltaSpeed = newSeparatingSpeed - separatingSpeed;
    float inverseInertia = contact.inverseLinearInertia();
    if(inverseInertia <= 0)
        return;

    float impulse = deltaSpeed / inverseInertia;
    Vector3 impulsePerIMass = contact.normal * impulse;

    contact.firstCollisionObject->applyLinearImpulse(impulsePerIMass);
    contact.secondCollisionObject->applyLinearImpulse(-impulsePerIMass);

    //printf("restitution %f\n", restitution);
    //printf("inverseInertia %f\n", inverseInertia);
    //printf("impulse %f\n", impulse);
}   

void PhysicsWorld::solveCollisionContactConstraintList(std::vector<ContactPoint> &contactList)
{
    if(contactList.empty())
        return;

    ContactPoint *bestFound = nullptr;

    for(auto &contact : contactList)
    {
        if(contact.hasCollisionResponse() && contact.inverseInertia() > 0.0)
        {
            //contact.update();
            if (!bestFound || contact.penetrationDistance > bestFound->penetrationDistance)
            {
                bestFound = &contact;
            }
        }
    }

    if(bestFound->penetrationDistance >= 0)
        solveCollisionContactConstraint(*bestFound);
}

void PhysicsWorld::solveCollisionContactConstraint(ContactPoint &contact)
{
    auto penetrationDistance = contact.penetrationDistance;
    //printf("penetrationDistance %f\n", penetrationDistance);
    if (penetrationDistance < 0)
        return;

    auto totalInverseMass = contact.inverseLinearInertia();
    if(totalInverseMass <= 0)
        return;

    auto movePerMass = contact.normal * (penetrationDistance / totalInverseMass);
    contact.firstCollisionObject->applyMovePerMass(movePerMass);
    contact.secondCollisionObject->applyMovePerMass(-movePerMass);

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