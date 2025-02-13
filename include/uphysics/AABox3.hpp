#ifndef UPHYSICS_BOX_HPP
#define UPHYSICS_BOX_HPP

#include "Vector3.hpp"
#include "TRSTransform.hpp"
#include "RigidTransform.hpp"
#include "Ray.hpp"
#include <vector>
#include <optional>

namespace UPhysics
{

struct AABox3
{
    AABox3()
        : min{0}, max(0) {}

    AABox3(Vector3 cmin, Vector3 cmax)
        : min{cmin}, max(cmax) {}

    static AABox3 empty()
    {
        AABox3 result;
        result.min = Vector3(INFINITY);
        result.max = Vector3(-INFINITY);
        return result;
    }

    static AABox3 forPoints(const std::vector<Vector3> &points)
    {
        auto result = empty();
        for(auto &p : points)
            result.insertPoint(p);

        return result;
    }

    void insertPoint(const Vector3 &p)
    {
        min = min.min(p);
        max = max.max(p);
    }

    void insertBox(const AABox3 &box)
    {
        min = min.min(box.min);
        max = max.max(box.max);
    }

    AABox3 unionWith(const AABox3 &box)
    {
        return AABox3{
            min.min(box.min),
            max.max(box.max)
        };
    }
    
    Vector3 extent() const
    {
        return max - min;
    }

    Vector3 halfExtent() const
    {
        return (max - min)*0.5f;
    }

    Vector3 center() const
    {
        return min + halfExtent();
    }

    bool isEmpty() const
    {
        return max.x > min.x || max.y > min.y || max.z > min.z;
    }

    bool isBoxOutside(const AABox3 &otherBox) const
    {
        return
            otherBox.max.x < min.x || max.x < otherBox.min.x ||
            otherBox.max.y < min.y || max.y < otherBox.min.y ||   
            otherBox.max.z < min.z || max.z < otherBox.min.z;
    }

    bool hasIntersectionWithBox(const AABox3 &otherBox) const
    {
        return !isBoxOutside(otherBox);
    }

    Vector3 supportInDirection(const Vector3 &d) const
    {
        return Vector3{
            d.x < 0 ? min.x : max.x,
            d.y < 0 ? min.y : max.y,
            d.z < 0 ? min.z : max.z
        };
    }

    AABox3 expandedWithMargin(float margin) const
    {
        return AABox3{
            min - margin, max + margin
        };
    }
    
    template<typename FT>
    void cornersDo(const FT &f)
    {
        f(Vector3(min.x, min.y, min.z));
        f(Vector3(min.x, min.y, max.z));
        f(Vector3(min.x, max.y, min.z));
        f(Vector3(min.x, max.y, max.z));

        f(Vector3(max.x, min.y, min.z));
        f(Vector3(max.x, min.y, max.z));
        f(Vector3(max.x, max.y, min.z));
        f(Vector3(max.x, max.y, max.z));
    }

    AABox3 transformedWith(const TRSTransform &transform)
    {
        auto result = empty();
        auto matrix = transform.asMatrix();
        cornersDo([&](const Vector3 &corner){
            result.insertPoint(((matrix*Vector4(corner, 1)).xyz()));
        });

        return result;
    }

    AABox3 transformedWith(const RigidTransform &transform)
    {
        auto result = empty();
        auto matrix = transform.asMatrix();
        cornersDo([&](const Vector3 &corner){
            result.insertPoint(((matrix*Vector4(corner, 1)).xyz()));
        });

        return result;
    }

    Vector3 computePenetrationNormalAtPoint(Vector3 point)
    {
        return computePenetrationNormalAndDistanceForPoint(point).first;
    }

    std::pair<Vector3, float> computePenetrationNormalAndDistanceForPoint(Vector3 point)
    {
        auto delta = point - center();
        auto deltaAbsolute = delta.abs()/ halfExtent();
        Vector3 normal{};

        if(deltaAbsolute.x >= deltaAbsolute.y) 
        {
            if(deltaAbsolute.x >= deltaAbsolute.z)
            {
                normal = Vector3(sign(delta.x));
            }
            else
            {
                normal = Vector3(0, 0, sign(delta.x));
            }
        } 
        else
        {
            if(deltaAbsolute.y >= deltaAbsolute.z)
            {
                normal =Vector3(0, sign(delta.y), 0);
            }
            else
            {
                normal =Vector3(0, 0, sign(delta.z));
            }
        }

        float penetrationDistance = abs((delta - (halfExtent()*normal)).dot(normal));

        return std::make_pair(normal, penetrationDistance);
    }

    std::optional<float> intersectionWithRay(const Ray &ray)
    {
        // Slab testing algorithm from: A Ray-Box Intersection Algorithm andEfficient Dynamic Voxel Rendering. By Majercik et al"
        auto t0 = (min - ray.origin)*ray.inverseDirection;
        auto t1 = (max - ray.origin)*ray.inverseDirection;
        auto tmin = t0.min(t1);
        auto tmax = t0.max(t1);
        auto maxTMin = std::max(std::max(std::max(tmin.x, tmin.y), tmin.z), ray.tmin);
        auto minTMax = std::min(std::min(std::min(tmax.x, tmax.y), tmax.z), ray.tmax);

        bool hasIntersection = maxTMin <= minTMax;
        if(!hasIntersection)
            return std::nullopt;

        return std::min(maxTMin, minTMax);
    }

    Vector3 min, max;
};

} // End of namespace UPhysics

#endif //UPHYSICS_BOX_HPP
