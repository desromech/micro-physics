#ifndef UPHYSICS_WORLD_HPP
#define UPHYSICS_WORLD_HPP


#include "CollisionObject.hpp"
#include "ContactManifold.hpp"
#include "Ray.hpp"

namespace UPhysics
{
typedef std::shared_ptr<class PhysicsWorld> PhysicsWorldPtr;
typedef std::shared_ptr<class RigidBody> RigidBodyPtr;

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
    void solveCollisionContactResponseList(const std::vector<ContactPointPtr> &contactList);
    void solveCollisionContactResponse(const ContactPointPtr &contact);
    void solveCollisionContactConstraintList(const std::vector<ContactPointPtr> &contactList);
    void solveCollisionContactConstraint(const ContactPointPtr &contact, float relaxationFactor);
    ContactPointPtr findMostSevereCollisionContactInList(const std::vector<ContactPointPtr> &contactList);
    ContactPointPtr findMostSeverePenetratingContactInList(const std::vector<ContactPointPtr> &contactList);

    template<typename FT>
    std::optional<ObjectRayCastingResult> rayCastForFirstThat(const Ray &ray, const FT &aPredicate)
    {
        std::optional<ObjectRayCastingResult> bestResult;

        for(size_t i = 0; i < collisionObjects.size(); ++i)
        {
            if(!aPredicate(collisionObjects[i]))
                continue;

            auto result = collisionObjects[i]->rayCast(ray);
            if(!result)
                continue;

            if(!bestResult || bestResult->distance > result->distance)
                bestResult = result;
        }

        return bestResult;
    }

    std::optional<ObjectRayCastingResult> rayCast(const Ray &ray)
    {
        std::optional<ObjectRayCastingResult> bestResult;

        for(size_t i = 0; i < collisionObjects.size(); ++i)
        {
            auto result = collisionObjects[i]->rayCast(ray);
            if(!result)
                continue;

            if(!bestResult || bestResult->distance > result->distance)
                bestResult = result;
        }

        return bestResult;
    }
    void addAwakeRigidBody(const RigidBodyPtr &RigidBody);

    void setGravity(const Vector3 &newGravity)
    {
        gravity = newGravity;
    }

    const Vector3 &getGravity() const
    {
        return gravity;
    }

    void addCollisionObject(const CollisionObjectPtr &collisionObject);
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
    std::vector<RigidBodyPtr> awakeRigidBodies;
    ContactManifoldCache contactManifoldCache;
};
} // End of namespace UPhysics

#endif //UPHYSICS_WORLD_HPP
