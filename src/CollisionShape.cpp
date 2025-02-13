#include "uphysics/CollisionShape.hpp"
#include "uphysics/GJK.hpp"
#include <stdio.h>

namespace UPhysics
{

std::optional<ShapeRayCastingResult> CollisionShape::rayCast(const Ray &ray)
{
    return std::nullopt;
}

std::vector<ContactPointPtr> CollisionShape::detectAndComputeCollisionContactPointsAt(const RigidTransform &firstTransform, const CollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    return std::vector<ContactPointPtr>{};
}

std::vector<ContactPointPtr> CollisionShape::detectAndComputeCollisionContactPointsWithConvexShapeAt(const RigidTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    return std::vector<ContactPointPtr>{};
}

std::vector<ContactPointPtr> CollisionShape::detectAndComputeCollisionContactPointsWithCompoundShape(const RigidTransform &firstTransform, const CompoundCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    return std::vector<ContactPointPtr>{};
}

std::optional<ShapeRayCastingResult> ConvexCollisionShape::rayCast(const Ray &ray)
{
    auto gjkResult = gjkRayCast(ray, [=](const Vector3 &D){
        return this->localSupportFunction(D);
    });
    if(!gjkResult)
        return std::nullopt;
    
    ShapeRayCastingResult result = {};
    result.distance = gjkResult->distance;
    result.normal = gjkResult->normal;
    result.shape = shared_from_this();
    return result;
}

std::vector<ContactPointPtr> ConvexCollisionShape::detectAndComputeCollisionContactPointsAt(const RigidTransform &firstTransform, const CollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    auto result = secondShape->detectAndComputeCollisionContactPointsWithConvexShapeAt(secondTransform, std::static_pointer_cast<ConvexCollisionShape> (shared_from_this()), firstTransform, -separatingAxisHint);
    for(auto &contact : result)
        contact = contact->flipped();
    return result;
}

std::vector<ContactPointPtr> ConvexCollisionShape::detectAndComputeCollisionContactPointsWithConvexShapeAt(const RigidTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    static const float ShallowPenetrationThreshold = 1.0e-5;

    auto &&firstSupportFunction = [&](const Vector3 &D) {
        return firstTransform.transformPosition(localSupportFunction(firstTransform.inverseTransformNormal(D)));
    };
    auto &&secondSupportFunction = [&](const Vector3 &D) {
        return secondTransform.transformPosition(secondShape->localSupportFunction(secondTransform.inverseTransformNormal(D)));
    };

    auto gjkSimplex = computeGJKSimplexFor(firstSupportFunction, secondSupportFunction, -separatingAxisHint);
    auto closestPointToOrigin = gjkSimplex.computeClosesPointToOrigin();
    auto totalMargin = margin + secondShape->margin;
    auto shapeDistance = closestPointToOrigin.length();
    if(shapeDistance > totalMargin)
        return std::vector<ContactPointPtr>{};

    if (shapeDistance >= ShallowPenetrationThreshold) {

        auto shallowContact = std::make_shared<ContactPoint> ();
        shallowContact->normal = closestPointToOrigin.normalized();
        shallowContact->requiredSeparation = totalMargin;
        shallowContact->firstPoint = gjkSimplex.computeClosesPointToOriginInFirstObject();
        shallowContact->secondPoint = gjkSimplex.computeClosesPointToOriginInSecondObject();
        shallowContact->computeLocalVersionsWithTransforms(firstTransform, secondTransform);
        shallowContact->computeWorldContactPointAndDistances();
        //printf("Shallow constant normal %f %f %f\n", shallowContact.normal.x, shallowContact.normal.y, shallowContact.normal.z);
        return std::vector{shallowContact};
    };

    auto contact = samplePenetrationSupportContact(firstSupportFunction, secondSupportFunction, totalMargin, separatingAxisHint);
    //printf("Sample penetration.\n");
    if(!contact)
        return std::vector<ContactPointPtr>{};

    //printf("Deep constant normal %f %f %f\n", contact.normal.x, contact.normal.y, contact.normal.z);
    contact->requiredSeparation = totalMargin;
    contact->computeLocalVersionsWithTransforms(firstTransform, secondTransform);
    contact->computeWorldContactPointAndDistances();
    return std::vector{contact};
}

std::vector<ContactPointPtr> ConvexCollisionShape::detectAndComputeCollisionContactPointsWithCompoundShape(const RigidTransform &firstTransform, const CompoundCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    return secondShape->detectAndComputeCollisionContactPointsWithConvexShapeAt(secondTransform, std::static_pointer_cast<ConvexCollisionShape> (shared_from_this()), firstTransform, separatingAxisHint);
}

void CompoundCollisionShape::addElement(const RigidTransform &transform, CollisionShapePtr shape)
{
    CompoundShapeElement element;
    element.transform = transform;
    element.shape = shape;
    elements.push_back(element);
    localBoundingBox.insertBox(element.shape->localBoundingBox);
    localBoundingBoxWithMargin = localBoundingBox.expandedWithMargin(margin);
}

std::vector<ContactPointPtr> CompoundCollisionShape::detectAndComputeCollisionContactPointsAt(const RigidTransform &firstTransform, const CollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    return secondShape->detectAndComputeCollisionContactPointsWithCompoundShape(secondTransform, std::static_pointer_cast<CompoundCollisionShape> (shared_from_this()), firstTransform, separatingAxisHint);
}
std::vector<ContactPointPtr> CompoundCollisionShape::detectAndComputeCollisionContactPointsWithConvexShapeAt(const RigidTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    std::vector<ContactPointPtr> contactPoints;
    for(auto &element : elements)
    {
        auto elementContactPoints = element.shape->detectAndComputeCollisionContactPointsWithConvexShapeAt(firstTransform, secondShape, secondTransform, separatingAxisHint);
        if(elementContactPoints.empty())
            continue;

        contactPoints.insert(contactPoints.end(), elementContactPoints.begin(), elementContactPoints.end());
    }
    return contactPoints;
}
std::vector<ContactPointPtr> CompoundCollisionShape::detectAndComputeCollisionContactPointsWithCompoundShape(const RigidTransform &firstTransform, const CompoundCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint)
{
    printf("TODO: Compund-Compund\n");
    return std::vector<ContactPointPtr> {};
}
} // End of namespace UPhysics
