#include "uphysics/CollisionShape.hpp"
#include "uphysics/GJK.hpp"
#include <stdio.h>

namespace UPhysics
{

std::vector<ContactPoint> CollisionShape::detectAndComputeCollisionContactPointsAt(const TRSTransform &firstTransform, const CollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    return std::vector<ContactPoint>{};
}

std::vector<ContactPoint> CollisionShape::detectAndComputeCollisionContactPointsWithConvexShapeAt(const TRSTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    return std::vector<ContactPoint>{};
}

std::vector<ContactPoint> ConvexCollisionShape::detectAndComputeCollisionContactPointsAt(const TRSTransform &firstTransform, const CollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    return secondShape->detectAndComputeCollisionContactPointsWithConvexShapeAt(secondTransform, std::static_pointer_cast<ConvexCollisionShape> (shared_from_this()), firstTransform, separatingAxisHint);
}

std::vector<ContactPoint> ConvexCollisionShape::detectAndComputeCollisionContactPointsWithConvexShapeAt(const TRSTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    static const float ShallowPenetrationThreshold = 1.0e-5;

    auto &&firstSupportFunction = [&](const Vector3 &D) {
        return firstTransform.transformPosition(localSupportFunction(firstTransform.inverseTransformNormal(D)));
    };
    auto &&secondSupportFunction = [&](const Vector3 &D) {
        return secondTransform.transformPosition(secondShape->localSupportFunction(secondTransform.inverseTransformNormal(D)));
    };

    auto gjkSimplex = computeGJKSimplexFor(firstSupportFunction, secondSupportFunction, separatingAxisHint);
    auto closestPointToOrigin = gjkSimplex.computeClosesPointToOrigin();
    auto totalMargin = margin + secondShape->margin;
    auto shapeDistance = closestPointToOrigin.length();
    if(shapeDistance > totalMargin)
        return std::vector<ContactPoint>{};

    if (shapeDistance >= ShallowPenetrationThreshold) {
        printf("Shallow Convex Convex contact.\n");
        ContactPoint shallowContact;
        shallowContact.normal = closestPointToOrigin.normalized();
        shallowContact.requiredSeparation = totalMargin;
        shallowContact.firstPoint = gjkSimplex.computeClosesPointToOriginInFirstObject();
        shallowContact.secondPoint = gjkSimplex.computeClosesPointToOriginInSecondObject();
        shallowContact.computeLocalVersionsWithTransforms(firstTransform, secondTransform);
        shallowContact.computeWorldContactPointAndDistances();
        return std::vector{shallowContact};
    };

    auto contact = samplePenetrationSupportContact(firstSupportFunction, secondSupportFunction, totalMargin, separatingAxisHint);
    printf("Sample penetration.\n");
    if(!contact.isValid)
        return std::vector<ContactPoint>{};
    return std::vector{contact};
}

} // End of namespace UPhysics
