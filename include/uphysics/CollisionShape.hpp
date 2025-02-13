#ifndef UPHYSICS_COLLISION_SHAPE_HPP
#define UPHYSICS_COLLISION_SHAPE_HPP

#include "Vector3.hpp"
#include "AABox3.hpp"
#include "ContactPoint.hpp"
#include "uphysics/Ray.hpp"
#include <vector>
#include <memory>
#include <optional>

namespace UPhysics
{

typedef std::shared_ptr<class CollisionShape> CollisionShapePtr;
typedef std::shared_ptr<class ConvexCollisionShape> ConvexCollisionShapePtr;
typedef std::shared_ptr<class CompoundCollisionShape> CompoundCollisionShapePtr;

struct ShapeRayCastingResult
{
    CollisionShapePtr shape;
    float distance;
    Vector3 normal;
};

class CollisionShape : public std::enable_shared_from_this<CollisionShape>
{
public:
    virtual bool isConvext() const
    {
        return false;
    }

    virtual std::optional<ShapeRayCastingResult> rayCast(const Ray &ray);

    virtual std::vector<ContactPointPtr> detectAndComputeCollisionContactPointsAt(const RigidTransform &firstTransform, const CollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint);
    virtual std::vector<ContactPointPtr> detectAndComputeCollisionContactPointsWithConvexShapeAt(const RigidTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint);
    virtual std::vector<ContactPointPtr> detectAndComputeCollisionContactPointsWithCompoundShape(const RigidTransform &firstTransform, const CompoundCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint);

    virtual Matrix3x3 computeInertiaTensorWithMass(float mass)
    {
        auto extent = localBoundingBox.extent();
        auto w = extent.x;
        auto h = extent.y;
        auto d = extent.z;
        auto m12 = mass / 12.0f;

        return Matrix3x3{
            {m12*(h*h +d*d), 0, 0},
            {0, m12*(w*w + h*h), 0},
            {0, 0, m12*(w*w + d*d)}
        };
    }

    void setLocalBoundingBox(const AABox3 &box)
    {
        localBoundingBox = box;
        localBoundingBoxWithMargin = localBoundingBox.expandedWithMargin(margin);
    }

    float margin = 0.01f;
    AABox3 localBoundingBox;
    AABox3 localBoundingBoxWithMargin;
};

class ConvexCollisionShape : public CollisionShape
{
public:
    virtual bool isConvext() const
    {
        return true;
    }

    virtual Vector3 localSupportFunction(const Vector3 &D) const
    {
        return Vector3::zeros();
    }

    virtual std::optional<ShapeRayCastingResult> rayCast(const Ray &ray) override;

    virtual std::vector<ContactPointPtr> detectAndComputeCollisionContactPointsAt(const RigidTransform &firstTransform, const CollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    virtual std::vector<ContactPointPtr> detectAndComputeCollisionContactPointsWithConvexShapeAt(const RigidTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    virtual std::vector<ContactPointPtr> detectAndComputeCollisionContactPointsWithCompoundShape(const RigidTransform &firstTransform, const CompoundCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    
};

class BoxCollisionShape : public ConvexCollisionShape
{
public:
    const Vector3 &getHalfExtent()
    {
        return halfExtent;
    }

    void setHalfExtent(Vector3 newHalfExtent)
    {
        halfExtent = newHalfExtent;
        localBoundingBox = AABox3(-halfExtent, halfExtent);
        localBoundingBoxWithMargin = localBoundingBox.expandedWithMargin(margin);
    }

    virtual std::optional<ShapeRayCastingResult> rayCast(const Ray &ray) override;

    virtual Matrix3x3 computeInertiaTensorWithMass(float mass)
    {
        auto w = halfExtent.x*2;
        auto h = halfExtent.y*2;
        auto d = halfExtent.z*2;
        auto m12 = mass / 12.0f;

        return Matrix3x3{
            {m12*(h*h +d*d), 0, 0},
            {0, m12*(w*w + h*h), 0},
            {0, 0, m12*(w*w + d*d)}
        };
    }

    virtual Vector3 localSupportFunction(const Vector3 &D) const override
    {
        return Vector3(
            D.x >= 0 ? halfExtent.x : -halfExtent.x,
            D.y >= 0 ? halfExtent.y : -halfExtent.y,
            D.z >= 0 ? halfExtent.z : -halfExtent.z
        );
    }

protected:
    Vector3 halfExtent = Vector3(0.5f, 0.5f, 0.5f);
};

class SphereCollisionShape : public ConvexCollisionShape
{
public:
    float getRadius() const
    {
        return radius;
    }

    void setRadius(float newRadius)
    {
        radius = newRadius;
        localBoundingBox = AABox3(Vector3(-radius,-radius,-radius), Vector3(radius, radius, radius));
        localBoundingBoxWithMargin = localBoundingBox.expandedWithMargin(margin);
    }

    virtual std::optional<ShapeRayCastingResult> rayCast(const Ray &ray) override;

    virtual Vector3 localSupportFunction(const Vector3 &D) const override
    {
        return D.safeNormalized() * radius;
    }

protected:
    float radius = 1.0f;
};

class ConvexHullCollisionShape : public ConvexCollisionShape
{
public:

    void addCorner(const Vector3 &corner)
    {
        corners.push_back(corner);
    }

    void computeBoundingBoxes()
    {
        localBoundingBox = AABox3::empty();
        for (auto &corner : corners)
            localBoundingBox.insertPoint(corner);
        localBoundingBoxWithMargin = localBoundingBox.expandedWithMargin(margin);
    }

    virtual Vector3 localSupportFunction(const Vector3 &D) const override
    {   
        float bestDistance = -INFINITY;
        auto bestFound = Vector3::zeros();
        for(auto &corner : corners)
        {
            auto distance = corner.dot(D);
            if (distance > bestDistance)
            {
                bestDistance = distance;
                bestFound = corner;
            }
        }
        return bestFound;
    }

    std::vector<Vector3> corners;
};

class HeightfieldCollisionShape : public CollisionShape
{
public:

};

class CompoundShapeElement
{
public:
    RigidTransform transform;
    CollisionShapePtr shape;
};

class CompoundCollisionShape : public CollisionShape
{
public:
    void addElement(const RigidTransform &transform, CollisionShapePtr shape);

    virtual std::optional<ShapeRayCastingResult> rayCast(const Ray &ray) override;

    virtual std::vector<ContactPointPtr> detectAndComputeCollisionContactPointsAt(const RigidTransform &firstTransform, const CollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    virtual std::vector<ContactPointPtr> detectAndComputeCollisionContactPointsWithConvexShapeAt(const RigidTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    virtual std::vector<ContactPointPtr> detectAndComputeCollisionContactPointsWithCompoundShape(const RigidTransform &firstTransform, const CompoundCollisionShapePtr &secondShape, const RigidTransform &secondTransform, const Vector3 &separatingAxisHint) override;

private:
    std::vector<CompoundShapeElement> elements;
};

}; // End of namespace UPhysics

#endif //UPHYSICS_COLLISION_SHAPE_HPP
