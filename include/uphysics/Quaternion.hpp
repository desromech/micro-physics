#ifndef UPHYSICS_QUATERNION_HPP
#define UPHYSICS_QUATERNION_HPP

#include <math.h>
#include "Matrix3x3.hpp"
#include "Vector4.hpp"
#include "Math.hpp"

namespace UPhysics
{

struct Quaternion
{
    Quaternion(float cw = 1.0f)
        : x(0), y(0), z(0), w(cw)
    {}

    Quaternion(float cx, float cy, float cz, float cw)
        : x(cx), y(cy), z(cz), w(cw)
    {}

    Quaternion(Vector3 v, float cw = 0.0f)
        : x(v.x), y(v.y), z(v.z), w(cw)
    {}

    static Quaternion identity()
    {
        return Quaternion(0, 0, 0, 1);
    }

    static Quaternion fromVector4(const Vector4 &v)
    {
        return Quaternion(v.x, v.y, v.z, v.w);
    }

    static Quaternion XRotation(float angle)
    {
        float halfAngle = angle * 0.5;
        float c = cos(halfAngle);
        float s = sin(halfAngle);
        return Quaternion(s, 0, 0, c);
    }

    static Quaternion YRotation(float angle)
    {
        float halfAngle = angle * 0.5;
        float c = cos(halfAngle);
        float s = sin(halfAngle);
        return Quaternion(0, s, 0, c);
    }

    static Quaternion ZRotation(float angle)
    {
        float halfAngle = angle * 0.5;
        float c = cos(halfAngle);
        float s = sin(halfAngle);
        return Quaternion(0, 0, s, c);
    }

    float dot(const Quaternion &o) const
    {
        return x*o.x + y*o.y + z*o.z + w*o.w;
    }
    float length2() const
    {
        return dot(*this);
    }

    float length() const
    {
        return sqrt(length2());
    }

    Quaternion conjugated() const
    {
        return Quaternion(-x, -y, -z, w);
    }

    Quaternion inverse() const
    {
        return conjugated() / length2();
    }

    Quaternion normalized() const
    {
        auto l = length();
        return Quaternion(x/l, y/l, z/l, w/l);
    }

    Quaternion safeNormalized() const
    {
        auto l = length();
        if(l < 1e-12)
            return Quaternion(0.0f);

        return Quaternion(x/l, y/l, z/l, w/l);
    }

    Quaternion interpolateTo(const Quaternion &o, float alpha) const
    {
        return Quaternion(
            mix(x, o.x, alpha),
            mix(y, o.y, alpha),
            mix(z, o.z, alpha),
            mix(w, o.w, alpha)
        ).normalized();
    }

    Quaternion exp() const
    {
        auto v = Vector3(x, y, z);
        auto vl = v.length();
        auto ew = ::exp(w);
        if(UPhysics::closeTo(vl, 0.0)) {
            return Quaternion(0, 0, 0, ew);
        }
        
        auto c = cos(vl);
        auto s = sin(vl);
        return Quaternion(v * (s / vl*ew), ew*c);
    }

    Quaternion ln() const
    {
	    auto l = length();
	    auto xyz = Vector3(x, y, z);
	    auto xyzl = xyz.length();

        auto xyzln = xyzl > 0 ? (xyz * (acos(w/l) / xyzl)) : Vector3::zeros();
        return Quaternion(xyzln, ::log(l));
    }

    Vector3 rotateVector(const Vector3 &v) const
    {
        auto rotated = (*this) * Quaternion(v.x, v.y, v.z, 0.0) * this->conjugated();
        return Vector3(rotated.x, rotated.y, rotated.z);
    }

    Quaternion operator+(const Quaternion &o) const
    {
        return Quaternion(x + o.x, y + o.y, z + o.z, w + o.w);
    }

    Quaternion operator-(const Quaternion &o) const
    {
        return Quaternion(x - o.x, y - o.y, z - o.z, w - o.w);
    }

    Quaternion operator*(const Quaternion &o) const
    {
        return Quaternion(
            (w * o.x) + (x * o.w) + (y * o.z) - (z * o.y),
            (w * o.y) - (x * o.z) + (y * o.w) + (z * o.x),
            (w * o.z) + (x * o.y) - (y * o.x) + (z * o.w),
            (w * o.w) - (x * o.x) - (y * o.y) - (z * o.z)
        );
    }

    Quaternion operator*(float s) const
    {
        return Quaternion(x*s, y*s, z*s, w*s);
    }

    Quaternion operator/(float s) const
    {
        return Quaternion(x/s, y/s, z/s, w/s);
    }

    bool closeTo(const Quaternion &o) const
    {
        return UPhysics::closeTo(x, o.x) && UPhysics::closeTo(y, o.y) && UPhysics::closeTo(z, o.z) && UPhysics::closeTo(w, o.w);
    }

    bool operator==(const Quaternion &o)
    {
        return x == o.x && y == o.y && z == o.z && w == o.w;
    }

    Matrix3x3 asMatrix3x3() const
    {
        auto r = w;
        auto i = x;
        auto j = y;
        auto k = z;
        return Matrix3x3::makeWithRows(
            Vector3{1.0f - (2.0f*j*j) - (2.0f*k*k),
            (2.0f*i*j) - (2.0f*k*r),
            (2.0f*i*k) + (2.0f*j*r)},

            Vector3{(2.0f*i*j) + (2.0f*k*r),
            1.0f - (2.0f*i*i) - (2.0f*k*k),
            (2.0f*j*k) - (2.0f*i*r)},

            Vector3{(2.0f*i*k) - (2.0f*j*r),
            (2.0f*j*k) + (2.0f*i*r),
            1.0f - (2.0f*i*i) - (2.0f*j*j)}
        );
    }

    float x, y, z, w;
};
} // End of namespace UPhysics

#endif //UPHYSICS_QUATERNION_HPP