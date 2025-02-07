#ifndef UPHYSICS_COLLISION_SHAPE_HPP
#define UPHYSICS_COLLISION_SHAPE_HPP

#include "Vector3.hpp"
#include "AABox3.hpp"
#include "ContactPoint.hpp"
#include <vector>
#include <memory>

namespace UPhysics
{

typedef std::shared_ptr<class CollisionShape> CollisionShapePtr;
typedef std::shared_ptr<class ConvexCollisionShape> ConvexCollisionShapePtr;
typedef std::shared_ptr<class CompoundCollisionShape> CompoundCollisionShapePtr;

class CollisionShape : public std::enable_shared_from_this<CollisionShape>
{
public:
    virtual bool isConvext() const
    {
        return false;
    }

    virtual std::vector<ContactPoint> detectAndComputeCollisionContactPointsAt(const TRSTransform &firstTransform, const CollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint);
    virtual std::vector<ContactPoint> detectAndComputeCollisionContactPointsWithConvexShapeAt(const TRSTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint);
    virtual std::vector<ContactPoint> detectAndComputeCollisionContactPointsWithCompoundShape(const TRSTransform &firstTransform, const CompoundCollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint);

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

    virtual std::vector<ContactPoint> detectAndComputeCollisionContactPointsAt(const TRSTransform &firstTransform, const CollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    virtual std::vector<ContactPoint> detectAndComputeCollisionContactPointsWithConvexShapeAt(const TRSTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    virtual std::vector<ContactPoint> detectAndComputeCollisionContactPointsWithCompoundShape(const TRSTransform &firstTransform, const CompoundCollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    
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
    TRSTransform transform;
    CollisionShapePtr shape;
};

class CompoundCollisionShape : public CollisionShape
{
public:
    void addElement(const TRSTransform &transform, CollisionShapePtr shape);

    virtual std::vector<ContactPoint> detectAndComputeCollisionContactPointsAt(const TRSTransform &firstTransform, const CollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    virtual std::vector<ContactPoint> detectAndComputeCollisionContactPointsWithConvexShapeAt(const TRSTransform &firstTransform, const ConvexCollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint) override;
    virtual std::vector<ContactPoint> detectAndComputeCollisionContactPointsWithCompoundShape(const TRSTransform &firstTransform, const CompoundCollisionShapePtr &secondShape, const TRSTransform &secondTransform, const Vector3 &separatingAxisHint) override;

private:
    std::vector<CompoundShapeElement> elements;
};

}; // End of namespace UPhysics

#endif //UPHYSICS_COLLISION_SHAPE_HPP
