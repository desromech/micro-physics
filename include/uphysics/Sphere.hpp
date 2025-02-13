#ifndef UPHYSICS_SPHERE_HPP
#define UPHYSICS_SPHERE_HPP

#include "Vector3.hpp"
#include "TRSTransform.hpp"
#include "RigidTransform.hpp"
#include "Ray.hpp"
#include <vector>
#include <optional>

namespace UPhysics
{
struct Sphere
{
    std::optional<float> intersectionWithRay(const Ray &ray)
    {
        //Ray sphere intersection formula from: https://viclw17.github.io/2018/07/16/raytracing-ray-sphere-intersection/"
        auto a = ray.direction.dot(ray.direction);
        auto b = 2.0 * (ray.direction.dot(ray.origin - center));
        auto c = (ray.origin - center).length2() - (radius*radius);
        
        auto delta = b*b - (4*a*c);
        if(delta < 0)
            return std::nullopt;

        auto deltaSqrt = sqrt(delta);
        auto t1 = (-b - deltaSqrt) / (2*a);	
        auto t2 = (-b + deltaSqrt) / (2*a);
        
        bool isT1Valid = ray.tmin <= t1 && t1 <= ray.tmax;
        bool isT2Valid = ray.tmin <= t2 && t2 <= ray.tmax;
        
        if(isT1Valid && isT2Valid)
            return std::min(t1, t2);

        if(isT1Valid)
            return t1;

        if(isT2Valid)
            return t2;

        return std::nullopt;
    }

    float radius = 0;
    Vector3 center;
};

} // End of namespace UPhysics

#endif // UPHYSICS_SPHERE_HPP