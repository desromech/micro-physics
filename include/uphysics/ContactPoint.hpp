#ifndef USGAR_CONTACT_POINTS_HPP
#define USGAR_CONTACT_POINTS_HPP

#include "Vector3.hpp"
#include "Matrix3x3.hpp"
#include "RigidTransform.hpp"
#include <memory>
#include <vector>
#include <unordered_map>
#include <assert.h>

namespace UPhysics
{
    class CollisionObject;
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
        uint32_t epoch = 0;
        bool isValid = false;

        bool hasCollisionResponse() const;
        float inverseLinearInertia() const;
        float inverseAngularInertia() const;
        float inverseInertia() const;
        Vector3 relativeFirstPoint() const;
        Vector3 relativeSecondPoint() const;
        float separationSpeed() const;
        float closingSpeed() const;

        void update();
        void computeLocalVersionsWithTransforms(const RigidTransform &firstTransform, const RigidTransform &secondTransform);
        void computeWorldVersionWithTransforms(const RigidTransform &firstTransform, const RigidTransform &secondTransform);
        void computeWorldContactPointAndDistances();
        Matrix3x3 computeContactSpaceMatrix() const;
        ContactPoint flipped() const;
    };
} // End of namespace Usgar

#endif // USGAR_CONTACT_POINTS_HPP