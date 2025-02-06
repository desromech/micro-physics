#ifndef UPHYSICS_WORLD_HPP
#define UPHYSICS_WORLD_HPP


#include "CollisionObject.hpp"
#include "ContactManifold.hpp"

namespace UPhysics
{
typedef std::shared_ptr<class PhysicsWorld> PhysicsWorldPtr;

class PhysicsWorld
{
public:

    void evaluateForceGeneratorWithDeltaTime(float deltaTimestep);
    void integrateMovementWithDeltaTime(float deltaTimestep);
    void detectAndResolveCollisionsWithDeltaTime(float deltaTimestep);
    void sendToSleepRestingObjects(float deltaTimestep);
    void updateTimestep(float deltaTimestep);

    std::vector<std::pair<CollisionObjectPtr, CollisionObjectPtr> > computeBroadphasePairCandidates();
    void computeNarrowPhaseContactsFromBroadphasePairs(const std::vector<std::pair<CollisionObjectPtr, CollisionObjectPtr> > &broadphasePairs );
    void detectNarrowPhaseCollisionOf(const CollisionObjectPtr &firstCollisionObject, const CollisionObjectPtr &secondCollisionObject);
    void resolveContactManifoldsCollisionsAndConstraints(const std::vector<ContactManifoldPtr> &manifolds);
    void solveCollisionContactResponseList(std::vector<ContactPoint> &contactList);
    void solveCollisionContactResponse(ContactPoint &contact);
    void solveCollisionContactConstraintList(std::vector<ContactPoint> &contactList);
    void solveCollisionContactConstraint(ContactPoint &contact);

    void setGravity(const Vector3 &newGravity)
    {
        gravity = newGravity;
    }

    const Vector3 &getGravity() const
    {
        return gravity;
    }

    void addCollisionObject(const CollisionObjectPtr &collisionObject)
    {
        if(collisionObject->ownerWorld == this)
            return;

        collisionObject->ownerWorld = this;
        collisionObjects.push_back(collisionObject);
    }

    void removeCollisionObject(const CollisionObjectPtr &collisionObject)
    {
        auto it = collisionObjects.begin();
        while(it != collisionObjects.end())
        {
            if(*it == collisionObject)
            {
                collisionObjects.erase(it);
                return;
            }
        }
    }

private:
    Vector3 gravity = Vector3(0, -9.8, 0);
    std::vector<CollisionObjectPtr> collisionObjects;
    ContactManifoldCache contactManifoldCache;
};
} // End of namespace UPhysics

#endif //UPHYSICS_WORLD_HPP
