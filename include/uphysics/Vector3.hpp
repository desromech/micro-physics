#ifndef UPHYSICS_VECTOR3_HPP
#define UPHYSICS_VECTOR3_HPP

#include "Vector2.hpp"
#include "Math.hpp"
#include <algorithm>
#include <math.h>
namespace UPhysics
{
struct Vector4;

struct alignas(16) Vector3
{
    float x, y, z;

    Vector3() = default;
    Vector3(const Vector2 &v, float cz)
        : x(v.x), y(v.y), z(cz) {}
    Vector3(float cx, float cy, float cz)
        : x(cx), y(cy), z(cz) {}
    Vector3(float s)
        : x(s), y(s), z(s) {}

    static Vector3 zeros()
    {
        return Vector3(0, 0, 0);
    }

    Vector3 min(const Vector3 &o) const
    {
        return Vector3(
                std::min(x, o.x),
                std::min(y, o.y),
                std::min(z, o.z)
        );
    }

    Vector3 max(const Vector3 &o) const
    {
        return Vector3(
                std::max(x, o.x),
                std::max(y, o.y),
                std::max(z, o.z)
        );
    }

    Vector3 ceil() const
    {
        return Vector3(::ceil(x), ::ceil(y), ::ceil(z));
    }

    Vector3 floor() const
    {
        return Vector3(::floor(x), ::floor(y), ::floor(z));
    }

    Vector3 cross(const Vector3 &o) const
    {
        return Vector3{
            y*o.z - z*o.y,
            z*o.x - x*o.z,
            x*o.y - y*o.x
        };
    }

    Vector3 reciprocal() const
    {
        return Vector3{1.0f / x, 1.0f / y, 1.0f / z};
    }

    float dot(const Vector3 &o) const
    {
        return x*o.x + y*o.y + z*o.z;
    }

    float length2() const
    {
        return this->dot(*this);
    }

    float length() const
    {
        return sqrt(length2());
    }

    Vector3 pow(const Vector3 exponents) const
    {
        return Vector3(::powf(x, exponents.x), ::powf(y, exponents.y), ::powf(z, exponents.z));
    }

    Vector3 normalized() const
    {
        return *this / length();
    }

    Vector3 abs() const
    {
        return Vector3(::abs(x), ::abs(y), ::abs(z));
    }

    Vector3 safeNormalized() const
    {
        auto l = length();
        if(l < 1e-12)
            return *this;

        return *this / length();
    }

    Vector3 safeReciprocal() const
    {
        return Vector3(safeNumberReciprocal(x), safeNumberReciprocal(y), safeNumberReciprocal(z));
    }

    Vector2 xy() const
    {
        return Vector2(x, y);
    }

    Vector3 rounded() const
    {
        return (*this + 0.5).floor();
    }

    Vector3 operator-() const
    {
        return Vector3{-x, -y, -z};
    }

    Vector3 operator+(const Vector3 &o) const
    {
        return Vector3{x + o.x, y + o.y, z + o.z};
    }

    Vector3 operator-(const Vector3 &o) const
    {
        return Vector3{x - o.x, y - o.y, z - o.z};
    }

    Vector3 operator*(const Vector3 &o) const
    {
        return Vector3{x * o.x, y * o.y, z * o.z};
    }

    Vector3 operator/(const Vector3 &o) const
    {
        return Vector3{x / o.x, y / o.y, z / o.z};
    }

    Vector3 operator+=(const Vector3 &o)
    {
        return *this = *this + o;
    }

    Vector3 operator*=(const Vector3 &o)
    {
        return *this = *this * o;
    }

    bool operator==(const Vector3 & o) const
    {
        return x == o.x && y == o.y && z == o.z;
    }

    bool operator!=(const Vector3 & o) const
    {
        return x != o.x || y != o.y || z != o.z;
    }

    bool closeTo(const Vector3 & o) const
    {
        return ::UPhysics::closeTo(x, o.x) && ::UPhysics::closeTo(y, o.y) && ::UPhysics::closeTo(z, o.z);
    }

    Vector4 asVector4() const;

    bool hasNaN() const
    {
        return isnan(x) || isnan(y) || isnan(z);
    }

    Vector3 operator/=(const Vector3 &o)
    {
        x /= o.x;
        x /= o.y;
        x /= o.z;
        return *this;
    }

};

struct PackedVector3
{
    PackedVector3() = default;
    PackedVector3(const Vector3 &v)
        : x(v.x), y(v.y), z(v.z) {}
    PackedVector3(float cx, float cy, float cz)
        : x(cx), y(cy), z(cz) {}

    Vector3 asVector3() const
    {
        return Vector3(x, y, z);
    }
    float x, y, z;
};

} // End of namespace UPhysics
#endif //UPHYSICS_VECTOR3_HPP