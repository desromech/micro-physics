#ifndef UPHYSICS_RAY_HPP
#define  UPHYSICS_RAY_HPP

#include "Vector3.hpp"
#include "RigidTransform.hpp"

namespace UPhysics
{
struct Ray
{
    static Ray fromTo(const Vector3 &startPoint, const Vector3 &endPoint)
    {
        auto vector = endPoint - startPoint;
        auto vectorLength = vector.length();
        auto direction = vector / (vectorLength != 0 ? vectorLength : 1);

        return Ray::withOriginDirectionTMinMax(startPoint, direction, 0, vectorLength);
    }

    static Ray withOriginDirectionTMinMax(const Vector3 &origin, const Vector3 &direction, float tmin, float tmax)
    {
        return Ray{
            origin,
            direction,
            direction.safeReciprocal(),
            tmin, tmax
        };
    }

    Vector3 pointAtDistance(float distance) const
    {
        return origin + direction*distance;
    }

    Ray transformedWith(const RigidTransform &transform) const
    {
        auto newOrigin = transform.transformPosition(origin);
        auto newDirection = transform.transformVector(direction);
        auto newTmin = tmin;
        auto newTmax = tmax;

        return Ray{
            newOrigin,
            newDirection,
            newDirection.safeReciprocal(),
            newTmin, newTmax
        };
    }

    Ray inverseTransformedWith(const RigidTransform &transform) const
    {
        auto newOrigin = transform.inverseTransformPosition(origin);
        auto newDirection = transform.inverseTransformVector(direction);
        auto newTmin = tmin;
        auto newTmax = tmax;

        return Ray{
            newOrigin,
            newDirection,
            newDirection.safeReciprocal(),
            newTmin, newTmax
        };
    }

    Vector3 origin;
    Vector3 direction;
    Vector3 inverseDirection;
    float tmin;
    float tmax;
};
} // End of namespace UPhysics
#endif 
