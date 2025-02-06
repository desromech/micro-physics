#ifndef UPHYSICS_RIGID_BODY_HPP
#define UPHYSICS_RIGID_BODY_HPP

#include "CollisionObject.hpp"

namespace UPhysics
{
class RigidBody : public CollisionObject
{
public:
    virtual float getMass() const override
    {
        return mass;
    }

    virtual float getMassReciprocal() const override
    {
        return inverseMass;
    }

private:
    float mass = 0.0f;
    float inverseMass = 0.0f;
};
} // End of namespace UPhysics

#endif //UPHYSICS_RIGID_BODY_HPP

