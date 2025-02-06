#ifndef UPHYSICS_VECTOR4_HPP
#define UPHYSICS_VECTOR4_HPP

#include <math.h>
#include "Vector3.hpp"

namespace UPhysics
{
struct alignas(16) Vector4
{
    float x, y, z, w;

    Vector4() = default;
    Vector4(float cx, float cy, float cz, float cw)
        : x(cx), y(cy), z(cz), w(cw) {}
    Vector4(Vector3 v, float cw)
        : x(v.x), y(v.y), z(v.z), w(cw) {}
    Vector4(float s)
        : x(s), y(s), z(s), w(s) {}

    Vector4 min(const Vector4 &o) const
    {
        return Vector4(
                std::min(x, o.x),
                std::min(y, o.y),
                std::min(z, o.z),
                std::min(w, o.w)
        );
    }

    Vector4 max(const Vector4 &o) const
    {
        return Vector4(
                std::max(x, o.x),
                std::max(y, o.y),
                std::max(z, o.z),
                std::max(w, o.w)
        );
    }

    Vector3 xyz() const
    {
        return Vector3(x, y, z);
    }

    float dot(const Vector4 &o) const
    {
        return x*o.x + y*o.y + z*o.z + w*o.w;
    }

    Vector4 operator-() const
    {
        return Vector4{-x, -y, -z, -w};
    }

    Vector4 operator+(const Vector4 &o) const
    {
        return Vector4{x + o.x, y + o.y, z + o.z, w + o.w};
    }

    Vector4 operator-(const Vector4 &o) const
    {
        return Vector4{x - o.x, y - o.y, z - o.z, w - o.w};
    }

    Vector4 operator*(const Vector4 &o) const
    {
        return Vector4{x * o.x, y * o.y, z * o.z, w * o.w};
    }

    Vector4 operator/(const Vector4 &o) const
    {
        return Vector4{x / o.x, y / o.y, z / o.z, w / o.w};
    }

    Vector3 minorAt(int index) const
    {
        switch(index)
        {
        case 0: return Vector3{y, z, w};
        case 1: return Vector3{x, z, w};
        case 2: return Vector3{x, y, w};
        case 3: return Vector3{x, y, z};
        default: abort();
        }
    }
};

inline Vector4 Vector3::asVector4() const
{
    return Vector4{x, y, z, 0.0f};
}

struct alignas(16) IVector4
{
    int32_t x, y, z, w;

    IVector4() = default;
    IVector4(int32_t cx, int32_t cy, int32_t cz, int32_t cw)
        : x(cx), y(cy), z(cz), w(cw) {}

};



} // End of namespace UPhysics
#endif //UPHYSICS_VECTOR4_HPP