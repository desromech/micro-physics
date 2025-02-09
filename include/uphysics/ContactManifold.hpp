#ifndef UPHYSICS_CONTACT_MANIFOLD_HPP
#define UPHYSICS_CONTACT_MANIFOLD_HPP

#include "ContactPoint.hpp"
#include "Matrix3x3.hpp"
#include "CollisionObject.hpp"
#include <memory>
#include <array>
#include <vector>
#include <unordered_map>
#include <assert.h>

namespace UPhysics
{
    typedef struct std::shared_ptr<struct ContactManifold> ContactManifoldPtr;
    struct ContactManifold
    {
        static const size_t MaxContactPoints = 1;

        ContactManifoldPtr flipped()
        {
            ContactManifoldPtr result;
            result->contactList.reserve(contactList.size());
            for (size_t i = 0; i < contactList.size(); ++i)
                result->contactList.push_back(contactList[i]->flipped());
            return result; 
        }

        void addPoint(const ContactPointPtr& contactPoint)
        {
            assert(!contactPoint->normal.hasNaN());
            contactList.push_back(contactPoint);
            contactList.back()->firstCollisionObject = firstCollisionObject.get();
            contactList.back()->secondCollisionObject = secondCollisionObject.get();
        }

        void setCollisionObjects(const CollisionObjectPtr &first, const CollisionObjectPtr &second)
        {
            firstCollisionObject = first;
            secondCollisionObject = second;
            for(size_t i = 0; i < contactList.size(); ++i)
            {
                contactList[i]->firstCollisionObject = first.get();
                contactList[i]->secondCollisionObject = second.get();
            }
        }

        bool hasCollisionResponse() const
        {
            return (firstCollisionObject && firstCollisionObject->hasCollisionResponse()) ||
                (secondCollisionObject && secondCollisionObject->hasCollisionResponse());
        }

        void update()
        {
            for(size_t i = 0; i < contactList.size() ; ++i)
            {
                contactList[i]->firstCollisionObject = firstCollisionObject.get();
                contactList[i]->secondCollisionObject = secondCollisionObject.get();
                contactList[i]->update();
            }
        }

        void expireContactsUntil(uint32_t expiredEpoch, uint32_t newEpoch)
        {
            float MaxSeparationTolerated = 0.01f;
            update();

            size_t destPosition = 0;
            for(size_t i = 0; i < contactList.size(); ++i)
            {
                bool isExpired = (-contactList[i]->penetrationDistance > MaxSeparationTolerated) || contactList[i]->epoch < expiredEpoch;
                if(!isExpired)
                    contactList[destPosition++] = contactList[i];
            }
            contactList.resize(destPosition);
        }

        CollisionObjectPtr firstCollisionObject;
        CollisionObjectPtr secondCollisionObject;

        Vector3 lastSeparatingAxis = Vector3(1, 0, 0);
        uint32_t epoch = 0;
        std::vector<ContactPointPtr> contactList;
    };

    class ContactManifoldCache
    {
    public:
        ContactManifoldCache()
        {
        }

        void beginEpoch()
        {
            ++epoch;
	        expireOldManifoldContacts();
        }

        uint32_t lastExpiredEpoch()
        {
            if(epoch >= 4)
                return epoch - 4;
            else
                return (uint32_t)-1;
        }

        void expireOldManifoldContacts()
        {
            auto expiredEpoch = lastExpiredEpoch();
            for (auto &manifold : manifolds)
                manifold->expireContactsUntil(expiredEpoch, epoch);

            manifoldDictionary.clear();
        }

        void expireOldManifolds()
        {
        }

        uint64_t composeObjectsID(const CollisionObjectPtr &first, const CollisionObjectPtr &second) const
        {
            return first->getID() | (uint64_t(second->getID()) << 32);
        }

        Vector3 lastSeparatingAxisFor(const CollisionObjectPtr &first, const CollisionObjectPtr &second)
        {
            auto id = composeObjectsID(first, second);
            auto it = manifoldDictionary.find(id);
            if(it != manifoldDictionary.end())
                return it->second->lastSeparatingAxis;

            auto centerSeparation = second->getPosition() - first->getPosition();
            if(centerSeparation.closeTo(Vector3::zeros()))
                return Vector3(1, 0, 0);

            return centerSeparation;
        }

        ContactManifoldPtr getOrCreateManifoldFor(const CollisionObjectPtr &first, const CollisionObjectPtr &second)
        {
            auto id = composeObjectsID(first, second);
            auto it = manifoldDictionary.find(id);
            if(it != manifoldDictionary.end())
                return it->second;

            auto newManifold = std::make_shared<ContactManifold> ();
            newManifold->firstCollisionObject = first;
            newManifold->secondCollisionObject = second;
            manifolds.push_back(newManifold);
            manifoldDictionary[id] = newManifold;
            return newManifold;
        }

        void insertContactsForWith(const std::vector<ContactPointPtr> &contacts, const CollisionObjectPtr &first, const CollisionObjectPtr &second)
        {
            if(contacts.empty())
                return;

            auto manifold = getOrCreateManifoldFor(first, second);
            for (auto &contact : contacts)
            {
                auto insertedContact = contact;
                insertedContact->epoch = epoch;
                manifold->addPoint(insertedContact);
                manifold->epoch = epoch;
            }

        }

        void endEpoch()
        {
            expireOldManifolds();
        }

        std::vector<ContactManifoldPtr> manifolds;
        std::unordered_map<uint64_t, ContactManifoldPtr> manifoldDictionary;
        uint32_t epoch = 0;
    };

} // End of namespace Usgar

#endif // USGAR_CONTACT_POINTS_HPP