#ifndef UPHYSICS_TRS_TRANSFORM_HPP
#define UPHYSICS_TRS_TRANSFORM_HPP

#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Matrix3x3.hpp"
#include "Matrix4x4.hpp"
#include "Math.hpp"
#include "RigidTransform.hpp"

namespace UPhysics
{

struct TRSTransform
{
    TRSTransform() = default;
    TRSTransform(const RigidTransform &rigidTransform)
        : rotation(rigidTransform.rotation), translation(rigidTransform.translation) {}
    TRSTransform(const Vector3 &initScale, const Quaternion &initRotation , const Vector3 &initTranslation)
        : scale(initScale), rotation(initRotation), translation(initTranslation) {}

    Vector3 scale = Vector3(1.0f);
    Quaternion rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    Vector3 translation = Vector3(0.0f);

    Vector3 transformPosition(const Vector3 &position) const
    {
        return rotation.rotateVector(scale*position) + translation;
    }

    Vector3 inverseTransformPosition(const Vector3 &position) const
    {
        return rotation.conjugated().rotateVector(position - translation) / scale;
    }

    Vector3 transformNormal(const Vector3 &normal) const
    {
        return rotation.rotateVector(normal / scale);
    }

    Vector3 inverseTransformNormal(const Vector3 &normal) const
    {
        return rotation.conjugated().rotateVector(normal) * scale;
    }

    TRSTransform interpolateTo(const TRSTransform &target, float alpha)
    {
        TRSTransform result {};
        result.scale = mix(scale, target.scale, alpha);
        result.rotation = rotation.interpolateTo(target.rotation, alpha);
        result.translation = mix(translation, target.translation, alpha);
        return result;
    }

    Matrix4x4 asMatrix() const
    {
        auto mat3 = rotation.asMatrix3x3() * Matrix3x3::scale(scale);
        return Matrix4x4::withMatrix3x3AndTranslation(mat3, translation);
    }

    Matrix4x4 asInverseMatrix() const
    {
        auto mat3 = Matrix3x3::scale(scale.reciprocal()) * rotation.conjugated().asMatrix3x3();
        return Matrix4x4::withMatrix3x3AndTranslation(mat3, mat3*(-translation));
    }
};

} // End of namespace UPhysics
#endif //UPHYSICS_TRS_TRANSFORM_HPP