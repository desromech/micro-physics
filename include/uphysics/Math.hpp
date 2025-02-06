#ifndef UPHYSICS_MATH_HPP
#define UPHYSICS_MATH_HPP

namespace UPhysics
{
    const float CloseToEpsilon = 1.0e-6;

    template<typename T>
    T mix(const T&a, const T&b, float alpha)
    {
        return a*(1.0 - alpha) + b*alpha;
    }

    inline float closeTo(float a, float b)
    {
        float delta = a - b;
        return -CloseToEpsilon <= delta && delta <= CloseToEpsilon;
    }

    inline float sign(float v)
    {
        if (v < 0)
            return -1;
        else if (v > 0)
            return 1;
        return 0;
    }
} // End of namespace UPhysics

#endif //UPHYSICS_MATH_HPP