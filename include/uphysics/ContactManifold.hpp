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

        ContactManifold flipped()
        {
            ContactManifold result;
            for (size_t i = 0; i < size; ++i)
                result.points[i] = points[i].flipped();
            result.size = size;
            return result; 
        }

        void augmentWith(const ContactManifold &augmentation)
        {
            for(size_t i = 0; i < augmentation.size; ++i)
                addPoint(augmentation.points[i]);
        }

        void makeRoomForPoint()
        {
            if(size < MaxContactPoints)
                return;
            
            for(size_t i = 1; i < MaxContactPoints; ++i)
                points[i - 1] = points[i];
            --size;
        }

        void addPoint(const ContactPoint& contactPoint)
        {
            assert(!contactPoint.normal.hasNaN());
            makeRoomForPoint();
            auto &targetPoint = points[size];
            targetPoint = contactPoint;
            targetPoint.firstCollisionObject = firstCollisionObject.get();
            targetPoint.secondCollisionObject = secondCollisionObject.get();
            ++size;
        }

        void setCollisionObjects(const CollisionObjectPtr &first, const CollisionObjectPtr &second)
        {
            firstCollisionObject = first;
            secondCollisionObject = second;
            for(size_t i = 0; i < size; ++i)
            {
                points[i].firstCollisionObject = first.get();
                points[i].secondCollisionObject = second.get();
            }
        }

        bool hasCollisionResponse() const
        {
            return (firstCollisionObject && firstCollisionObject->hasCollisionResponse()) ||
                (secondCollisionObject && secondCollisionObject->hasCollisionResponse());
        }

        void update()
        {
            for(size_t i = 0; i <MaxContactPoints; ++i)
            {
                points[i].firstCollisionObject = firstCollisionObject.get();
                points[i].secondCollisionObject = secondCollisionObject.get();
                points[i].update();
            }
        }

        CollisionObjectPtr firstCollisionObject;
        CollisionObjectPtr secondCollisionObject;

        Vector3 lastSeparatingAxis = Vector3(1, 0, 0);

        size_t size = 0;
        std::array<ContactPoint, MaxContactPoints> points;
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

        void expireOldManifoldContacts()
        {
            manifolds.clear();
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

        void insertContactsForWith(const std::vector<ContactPoint> &contacts, const CollisionObjectPtr &first, const CollisionObjectPtr &second)
        {
            if(contacts.empty())
                return;

            auto manifold = getOrCreateManifoldFor(first, second);
            for (auto &contact : contacts)
            {
                auto insertedContact = contact;
                insertedContact.epoch = epoch;
                manifold->addPoint(insertedContact);
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