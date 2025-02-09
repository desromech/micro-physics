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
    /*std::vector<ContactPoint> contactList;
    for(auto &manifold : manifolds)
    {
        if(manifold->hasCollisionResponse())
        {
            contactList.insert(contactList.end(), manifold->points.begin(), manifold->points.begin() + manifold->size);
        }
    }*/

    for(auto &manifold : manifolds)
    {
        if(!manifold->hasCollisionResponse())
            continue;;

        solveCollisionContactResponseList(manifold->contactList);
        solveCollisionContactConstraintList(manifold->contactList);

    }


}

ContactPointPtr PhysicsWorld::findMostSevereCollisionContactInList(const std::vector<ContactPointPtr> &contactList)
{
    if(contactList.empty())
        return nullptr;

    ContactPointPtr bestFound = nullptr;
    float bestFoundClosingSpeed = -INFINITY;

    for(auto &contact : contactList)
    {
        if(contact->hasCollisionResponse() && contact->inverseInertia() > 0.0)
        {
            auto closingSpeed = contact->closingSpeed();
            if (!bestFound || closingSpeed > bestFoundClosingSpeed)
            {
                bestFound = contact;
                bestFoundClosingSpeed = closingSpeed;
            }
        }
    }
    return bestFound;

}

ContactPointPtr PhysicsWorld::findMostSeverePenetratingContactInList(const std::vector<ContactPointPtr> &contactList)
{
    if(contactList.empty())
        return nullptr;

    ContactPointPtr bestFound = nullptr;
    for (auto &contact : contactList)
    {
        if(contact->hasCollisionResponse() && contact->inverseLinearInertia() > 0.0) 
        {
            contact->update();
            if(!bestFound || contact->penetrationDistance > bestFound->penetrationDistance)
            {
                bestFound = contact;
            }
        }
    }

	return bestFound;
}

void PhysicsWorld::solveCollisionContactResponseList(const std::vector<ContactPointPtr> &contactList)
{
    if(contactList.empty())
        return;

    for(auto &contact : contactList)
        contact->update();

    for(size_t i = 0; i < contactList.size()*2; ++i)
    {
        ContactPointPtr contact = findMostSevereCollisionContactInList(contactList);
        if(!contact)
            break;
        solveCollisionContactResponse(contact);
        contact->update();
    }
}

void PhysicsWorld::solveCollisionContactResponse(const ContactPointPtr &contact)
{
#if 0
	// See Milling. 'Game Physics Engine Development'. Chapter 14 for details on these equations and the associated algorithms."
    auto firstCollisionObject = contact->firstCollisionObject;
    auto secondCollisionObject = contact->secondCollisionObject;

    auto contactNormal = contact->normal;

    auto relativeFirstPoint = contact->relativeFirstPoint();
    auto relativeSecondPoint = contact->relativeSecondPoint();

    auto contactLocalToWorldMatrix3x3 = contact->computeContactSpaceMatrix();

    auto velocityChangePerImpulseWorldMatrix = 
        firstCollisionObject->computeVelocityPerImpulseWorldMatrixForRelativeContactPoint(relativeFirstPoint)
        + secondCollisionObject->computeVelocityPerImpulseWorldMatrixForRelativeContactPoint(relativeSecondPoint);

    auto velocityChangePerImpulseContactMatrix = contactLocalToWorldMatrix3x3.transposed() * velocityChangePerImpulseWorldMatrix * contactLocalToWorldMatrix3x3;

    auto inverseMass = firstCollisionObject->getMassReciprocal() + secondCollisionObject->getMassReciprocal();

    velocityChangePerImpulseContactMatrix = velocityChangePerImpulseContactMatrix + Matrix3x3::scale(inverseMass);

    if (velocityChangePerImpulseContactMatrix.determinant() == 0) return;

	auto impulseChangePerVelocityContactMatrix = velocityChangePerImpulseContactMatrix.inverse();

	auto firstContactVelocity = firstCollisionObject->velocityAtRelativePoint(relativeFirstPoint);
	auto secondContactVelocity = secondCollisionObject->velocityAtRelativePoint(relativeSecondPoint);
	
	auto relativeSeparationVelocity = firstContactVelocity - secondContactVelocity;
	
	auto relativeContactSeparationVelocity = relativeSeparationVelocity * contactLocalToWorldMatrix3x3;
	if(relativeContactSeparationVelocity.x > 0.0)
        return;

	auto relativeVelocityFromIntegrationDelta = firstCollisionObject->getLinearVelocityIntegrationDelta() - secondCollisionObject->getLinearVelocityIntegrationDelta();
	auto relativeContactVelocityFromIntegrationDelta = relativeVelocityFromIntegrationDelta.dot(contactNormal);
	
	auto restitutionCoefficient = firstCollisionObject->getRestitutionCoefficient() * secondCollisionObject->getRestitutionCoefficient();
	
    const float restingContactVelocityLimit = 0.1;
	// Resting contact: reduce contact velocity by acceleration only speed increase, and set the restitution coeffiecient to 0"
	if (abs(relativeContactSeparationVelocity.x) < restingContactVelocityLimit)
		restitutionCoefficient = 0.0;

	auto deltaVelocity = -relativeContactSeparationVelocity.x - (restitutionCoefficient * (relativeContactSeparationVelocity.x - relativeContactVelocityFromIntegrationDelta));
	
	auto contactLocalVelocityChange = Vector3(deltaVelocity,
		-relativeContactSeparationVelocity.y,
		-relativeContactSeparationVelocity.z
    );
		
	auto contactLocalImpulse = impulseChangePerVelocityContactMatrix * contactLocalVelocityChange;

	// Compute the planar length for simulating friction.
	auto staticFrictionCoefficient = std::min(firstCollisionObject->getStaticFrictionCoefficient(), secondCollisionObject->getStaticFrictionCoefficient());
	auto planarImpulse = sqrt(contactLocalImpulse.y*contactLocalImpulse.y + contactLocalImpulse.z*contactLocalImpulse.z);

	// Is this in the limits for the static friction?
	if(planarImpulse > (contactLocalImpulse.x * staticFrictionCoefficient))
    {
		auto dynamicFrictionCoefficient = std::min(firstCollisionObject->getDynamicFrictionCoefficient(), secondCollisionObject->getDynamicFrictionCoefficient());
		
		contactLocalImpulse.y /= planarImpulse;
		contactLocalImpulse.z /= planarImpulse;

		//"contactLocalImpulse yz length = dynamicFrictionCoefficient * contactLocalImpulse x"
		
		//"CHECK ME: What is the meaning of this correction? [From Millington Game Physics Engine Development, Chapter 15 pp 410]"
		auto frictionNormalDelta = velocityChangePerImpulseContactMatrix.firstRow().dot(Vector3(1, dynamicFrictionCoefficient*contactLocalImpulse.y, dynamicFrictionCoefficient*contactLocalImpulse.z));
		contactLocalImpulse.x = deltaVelocity / frictionNormalDelta;

		contactLocalImpulse.y = contactLocalImpulse.y * dynamicFrictionCoefficient * contactLocalImpulse.x;
		contactLocalImpulse.z = contactLocalImpulse.z * dynamicFrictionCoefficient * contactLocalImpulse.x;
    }

	auto contactImpulse = contactLocalToWorldMatrix3x3 * contactLocalImpulse;
    //printf("contactLocalImpulse %f %f %f\n", contactLocalImpulse.x, contactLocalImpulse.y, contactLocalImpulse.z);

	if (firstCollisionObject->hasCollisionResponse())
        firstCollisionObject->applyImpulseAtRelativePosition(contactImpulse, relativeFirstPoint);
	if (secondCollisionObject->hasCollisionResponse())
        secondCollisionObject->applyImpulseAtRelativePosition(-contactImpulse, relativeSecondPoint);

#else
    // Are they already separating?
    auto separatingSpeed = contact->separationSpeed();
    //printf("separationSpeed %f\n", separatingSpeed);
    if(separatingSpeed > 0)
        return;

    auto restitution = std::min(contact->firstCollisionObject->getRestitutionCoefficient(), contact->secondCollisionObject->getRestitutionCoefficient());
    
    auto newSeparatingSpeed = -separatingSpeed*restitution;
    auto deltaSpeed = newSeparatingSpeed - separatingSpeed;
    float inverseInertia = contact->inverseLinearInertia();
    if(inverseInertia <= 0)
        return;

    float impulse = deltaSpeed / inverseInertia;
    Vector3 impulsePerIMass = contact->normal * impulse;

    contact->firstCollisionObject->applyLinearImpulse(impulsePerIMass);
    contact->secondCollisionObject->applyLinearImpulse(-impulsePerIMass);

    //printf("restitution %f\n", restitution);
    //printf("inverseInertia %f\n", inverseInertia);
    //printf("impulse %f\n", impulse);
#endif
}

void PhysicsWorld::solveCollisionContactConstraintList(const std::vector<ContactPointPtr> &contactList)
{
    if(contactList.empty())
        return;

    for(size_t i = 0; i <contactList.size()*2; ++i)
    {
        ContactPointPtr nextPoint = findMostSeverePenetratingContactInList(contactList);
        if(!nextPoint)
            break;

        solveCollisionContactConstraint(nextPoint, 0.8f); 
    }
    /*for(auto &contact : contactList)
        solveCollisionContactConstraint(contact, 0.8f);*/
}

void PhysicsWorld::solveCollisionContactConstraint(const ContactPointPtr &contact, float relaxationFactor)
{
    auto penetrationDistance = contact->penetrationDistance;
    //printf("penetrationDistance %f\n", penetrationDistance);
    if (penetrationDistance < 0)
        return;

    auto inverseInertia = contact->inverseInertia();
    if(inverseInertia <= 0)
        return;

    auto penetrationDelta = penetrationDistance * relaxationFactor / inverseInertia;
    if(contact->firstCollisionObject->hasCollisionResponse())
        contact->firstCollisionObject->applyMovementAtRelativePoint(penetrationDelta, contact->relativeFirstPoint(), contact->normal);

    if(contact->secondCollisionObject->hasCollisionResponse())
        contact->secondCollisionObject->applyMovementAtRelativePoint(penetrationDelta, contact->relativeSecondPoint(), -contact->normal);
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