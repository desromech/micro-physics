#include "uphysics/ContactPoint.hpp"
#include "uphysics/CollisionObject.hpp"

namespace UPhysics
{
bool ContactPoint::hasCollisionResponse() const
{
    return firstCollisionObject->hasCollisionResponse() || secondCollisionObject->hasCollisionResponse();
}

float ContactPoint::inverseLinearInertia() const
{
    return firstCollisionObject->getMassReciprocal() + secondCollisionObject->getMassReciprocal();
}

float ContactPoint::inverseAngularInertia() const
{
    return firstCollisionObject->computeAngularInertiaForRelativeContactPoint(relativeFirstPoint(), normal)
        + secondCollisionObject->computeAngularInertiaForRelativeContactPoint(relativeSecondPoint(), normal);
}

float ContactPoint::inverseInertia() const
{
    return inverseLinearInertia() + inverseAngularInertia();
}

Vector3 ContactPoint::relativeFirstPoint() const
{
    return firstPoint - firstCollisionObject->getPosition();
}

Vector3 ContactPoint::relativeSecondPoint() const
{
    return secondPoint - secondCollisionObject->getPosition();
}

float ContactPoint::separationSpeed() const
{
    return (firstCollisionObject->computeVelocityAtRelativePoint(relativeFirstPoint()) -
        secondCollisionObject->computeVelocityAtRelativePoint(relativeSecondPoint())).dot(normal);
}

float ContactPoint::closingSpeed() const
{
    return -separationSpeed();
}
void ContactPoint::update()
{
    //computeWorldVersionWithTransforms(firstCollisionObject->getTransform(),  secondCollisionObject->getTransform());
}

void ContactPoint::computeLocalVersionsWithTransforms(const RigidTransform &firstTransform, const RigidTransform &secondTransform)
{
    localFirstPoint = firstTransform.inverseTransformPosition(firstPoint);
    localSecondPoint = secondTransform.inverseTransformPosition(secondPoint);
    localFirstNormal = firstTransform.inverseTransformNormal(secondTransform.inverseTransformNormal(normal));
    localSecondNormal = secondTransform.inverseTransformNormal(firstTransform.inverseTransformNormal(-normal));
    assert(!localFirstNormal.hasNaN());
    assert(!localSecondNormal.hasNaN());
}

void ContactPoint::computeWorldVersionWithTransforms(const RigidTransform &firstTransform, const RigidTransform &secondTransform)
{
    firstPoint = firstTransform.transformPosition(localFirstPoint);
    secondPoint = firstTransform.transformPosition(localSecondPoint);
    normal = firstTransform.transformNormal(secondTransform.transformNormal(localFirstNormal));
    //normal = secondTransform.transformNormal(firstTransform.transformNormal(localFirstNormal));
    assert(!firstPoint.hasNaN());
    assert(!secondPoint.hasNaN());
    assert(!normal.hasNaN());
    computeWorldContactPointAndDistances();
}
void ContactPoint::computeWorldContactPointAndDistances()
{
    penetrationDistance = requiredSeparation + (secondPoint - firstPoint).dot(normal);
}

Matrix3x3 ContactPoint::computeContactSpaceMatrix() const
{
    Vector3 x = normal;
    Vector3 y = abs(normal.x) > abs(normal.y) ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
    Vector3 z = x.cross(y).safeNormalized();
    y = z.cross(x).safeNormalized();

    return Matrix3x3(x, y, z);
}

ContactPoint ContactPoint::flipped() const
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

} // End of namespace UPhysics