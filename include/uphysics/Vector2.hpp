#ifndef UPHYSICS_VECTOR2_HPP
#define UPHYSICS_VECTOR2_HPP

namespace UPhysics
{

struct alignas(8) Vector2
{
    Vector2() = default;
    Vector2(float cx, float cy)
        : x(cx), y(cy) {}
    Vector2(float s)
        : x(s), y(s) {}

    float x, y;

    static Vector2 zeros()
    {
        return Vector2(0.0f, 0.0f);
    }

    float dot(const Vector2 &o) const
    {
        return x*o.x + y*o.y;
    }

    Vector2 reciprocal() const
    {
        return Vector2(1.0f/x, 1.0f/y);
    }

    Vector2 operator-() const
    {
        return Vector2{-x, -y};
    }

    Vector2 operator+(const Vector2 &o) const
    {
        return Vector2{x + o.x, y + o.y};
    }

    Vector2 operator-(const Vector2 &o) const
    {
        return Vector2{x - o.x, y - o.y};
    }

    Vector2 operator*(const Vector2 &o) const
    {
        return Vector2{x * o.x, y * o.y};
    }

    Vector2 operator/(const Vector2 &o) const
    {
        return Vector2{x / o.x, y / o.y};
    }

    Vector2 operator+=(const Vector2 &o)
    {
        return *this = *this + o;
    }

    Vector2 operator-=(const Vector2 &o)
    {
        return *this = *this - o;
    }

};

} // End of namespace UPhysics
#endif //UPHYSICS_VECTOR2_HPP