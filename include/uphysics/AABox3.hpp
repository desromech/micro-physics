#ifndef UPHYSICS_BOX_HPP
#define UPHYSICS_BOX_HPP

#include "Vector3.hpp"
#include "TRSTransform.hpp"
#include <vector>

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
            otherBox.max.z < min.z || max.y < otherBox.min.z;
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

    Vector3 min, max;
};

} // End of namespace UPhysics

#endif //UPHYSICS_BOX_HPP
