#ifndef USGAR_CONTACT_POINTS_HPP
#define USGAR_CONTACT_POINTS_HPP

#include "Vector3.hpp"
#include "Matrix3x3.hpp"
#include "CollisionObject.hpp"
#include <memory>
#include <vector>
#include <unordered_map>
#include <assert.h>

namespace UPhysics
{
    typedef struct std::shared_ptr<struct ContactManifold> ContactManifoldPtr;

    struct ContactPoint
    {
        CollisionObject *firstCollisionObject = nullptr;
        CollisionObject *secondCollisionObject = nullptr;
        float requiredSeparation = 0;
        float penetrationDistance = 0;
        Vector3 normal = Vector3(0, 0, 0);
        Vector3 firstPoint = Vector3(0, 0, 0);
        Vector3 secondPoint = Vector3(0, 0, 0);
        Vector3 localFirstPoint = Vector3(0, 0, 0);
        Vector3 localSecondPoint = Vector3(0, 0, 0);
        Vector3 localFirstNormal = Vector3(0, 0, 0);
        Vector3 localSecondNormal = Vector3(0, 0, 0);
        bool isValid = false;

        bool hasCollisionResponse() const
        {
            return firstCollisionObject->hasCollisionResponse() || secondCollisionObject->hasCollisionResponse();
        }

        float inverseLinearInertia() const
        {
            return firstCollisionObject->getMassReciprocal() + secondCollisionObject->getMassReciprocal();
        }

        float inverseAngularInertia() const
        {
            return 0.0;
        }

        float inverseInertia() const
        {
            return inverseLinearInertia() + inverseAngularInertia();
        }

        Vector3 relativeFirstPoint() const
        {
            return firstPoint - firstCollisionObject->getPosition();
        }

        Vector3 relativeSecondPoint() const
        {
            return secondPoint - secondCollisionObject->getPosition();
        }

        float separationSpeed() const
        {
            return (firstCollisionObject->computeVelocityAtRelativePoint(relativeFirstPoint()) -
                secondCollisionObject->computeVelocityAtRelativePoint(relativeSecondPoint())).dot(normal);
        }

        float closingSpeed() const
        {
            return -separationSpeed();
        }

        void update()
        {
            computeWorldVersionWithTransforms(firstCollisionObject->getTransform(),  secondCollisionObject->getTransform());
        }

        void computeLocalVersionsWithTransforms(const TRSTransform &firstTransform, const TRSTransform &secondTransform)
        {
            localFirstPoint = firstTransform.inverseTransformPosition(firstPoint);
            localSecondPoint = secondTransform.inverseTransformPosition(secondPoint);
            localFirstNormal = firstTransform.inverseTransformNormal(secondTransform.inverseTransformNormal(normal));
            localSecondNormal = secondTransform.inverseTransformNormal(firstTransform.inverseTransformNormal(-normal));
            assert(!localFirstNormal.hasNaN());
            assert(!localSecondNormal.hasNaN());
        }
        void computeWorldVersionWithTransforms(const TRSTransform &firstTransform, const TRSTransform &secondTransform)
        {
            firstPoint = firstTransform.transformPosition(localFirstPoint);
            secondPoint = firstTransform.transformPosition(localSecondPoint);
            normal = firstTransform.transformNormal(secondTransform.transformNormal(localFirstNormal));
            assert(!firstPoint.hasNaN());
            assert(!secondPoint.hasNaN());
            assert(!normal.hasNaN());
            computeWorldContactPointAndDistances();
        }
        void computeWorldContactPointAndDistances()
        {
            penetrationDistance = requiredSeparation + (secondPoint - firstPoint).dot(normal);
        }
	
        Matrix3x3 computeContactSpaceMatrix() const
        {
            Vector3 x = normal;
            Vector3 y = abs(normal.x) > abs(normal.y) ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
            Vector3 z = x.cross(y).safeNormalized();
            y = z.cross(x).safeNormalized();

            return Matrix3x3(x, y, z);
        }


        ContactPoint flipped() const
        {
            auto result = *this;
            result.firstCollisionObject = secondCollisionObject;
            result.secondCollisionObject = firstCollisionObject;
            result.normal = -normal;
            result.firstPoint = secondPoint;
            result.secondPoint = firstPoint;
            result.isValid = isValid;
            result.localFirstPoint = localSecondPoint;
            result.localSecondPoint = localFirstPoint;
            result.localFirstNormal = localSecondNormal;
            result.localSecondNormal = localFirstNormal;
            return result;
        }
    };

    struct ContactManifold
    {
        static const size_t MaxContactPoints = 4;

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

        size_t size = 0;
        std::array<ContactPoint, MaxContactPoints> points;
    };

    class ContactManifoldCache
    {
    public:
        ContactManifoldCache()
        {
        }

        ContactManifoldPtr augmentCachedVersionWith(const ContactManifold &augmentation)
        {
            auto composedID = (uint64_t)augmentation.firstCollisionObject->getID() | ((uint64_t)augmentation.secondCollisionObject->getID() << 32);
            auto it = contactManifoldDictionary.find(composedID);
            if(it != contactManifoldDictionary.end())
            {
                auto cachedManifold = it->second;
                cachedManifold->augmentWith(augmentation);
                return cachedManifold;
            }

            auto newCachedManifold = std::make_shared<ContactManifold> ();
            *newCachedManifold = augmentation;
            contactManifoldList.push_back(newCachedManifold);
            contactManifoldDictionary.insert(std::make_pair(composedID, newCachedManifold));

            return newCachedManifold;
        }
        std::vector<ContactManifoldPtr> contactManifoldList;
        std::unordered_map<uint64_t, ContactManifoldPtr> contactManifoldDictionary;
    };

} // End of namespace Usgar

#endif // USGAR_CONTACT_POINTS_HPP