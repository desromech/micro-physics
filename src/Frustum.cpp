#include "Frustum.hpp"
#include "Plane.hpp"

namespace UPhysics
{

void Frustum::makeOrtho(float left, float right, float bottom, float top, float nearDistance, float farDistance)
{
    leftBottomNear = Vector3(left, bottom, -nearDistance);
	rightBottomNear = Vector3(right, bottom, -nearDistance);
	leftTopNear = Vector3(left, top, -nearDistance);
	rightTopNear = Vector3(right, top, -nearDistance);

    leftBottomFar = Vector3(left, bottom, -farDistance);
	rightBottomFar = Vector3(right, bottom, -farDistance);
	leftTopFar = Vector3(left, top, -farDistance);
	rightTopFar = Vector3(right, top, -farDistance);
    computePlanes();
    computeBoundingBox();
}

void Frustum::makeFrustum(float left, float right, float bottom, float top, float nearDistance, float farDistance)
{
    leftBottomNear = Vector3(left, bottom, -nearDistance);
	rightBottomNear = Vector3(right, bottom, -nearDistance);
	leftTopNear = Vector3(left, top, -nearDistance);
	rightTopNear = Vector3(right, top, -nearDistance);
	
    auto factor = farDistance / nearDistance;
    leftBottomFar = leftBottomNear * factor;
	rightBottomFar = rightBottomNear * factor;
	leftTopFar = leftTopNear * factor;
	rightTopFar = rightTopNear * factor;
    computePlanes();
    computeBoundingBox();
}
void Frustum::makePerspective(float fovy, float aspect, float nearDistance, float farDistance)
{
    float fovyRad = fovy *0.5f * (M_PI / 180.0f);
    float top = nearDistance * tan(fovyRad);
    float right = top * aspect;

    return makeFrustum(-right, right, -top, top, nearDistance, farDistance);
}

void Frustum::computePlanes()
{
    // Near
    planes[0] = Plane::makeWithPoints(leftBottomNear, rightBottomNear, leftTopNear);
    // Far
    planes[1] = Plane::makeWithPoints(rightBottomFar, leftBottomFar,leftTopFar);
    // Left
    planes[2] = Plane::makeWithPoints(leftBottomNear, leftTopNear, leftTopFar);
    // Right
    planes[3] = Plane::makeWithPoints(rightTopNear, rightBottomNear,rightTopFar);
    // Bottom
    planes[4] = Plane::makeWithPoints(rightBottomNear, leftBottomNear, leftBottomFar);
    // Top
    planes[5] = Plane::makeWithPoints(leftTopNear, rightTopNear, leftTopFar);
}


void Frustum::computeBoundingBox()
{
    boundingBox = AABox3::empty();
    boundingBox.insertPoint(leftBottomNear);
	boundingBox.insertPoint(rightBottomNear);
	boundingBox.insertPoint(leftTopNear);
	boundingBox.insertPoint(rightTopNear);
	boundingBox.insertPoint(leftBottomFar);
	boundingBox.insertPoint(rightBottomFar);
	boundingBox.insertPoint(leftTopFar);
	boundingBox.insertPoint(rightTopFar);
}

Frustum Frustum::splitAtNearAndFarLambda(float nearLambda, float farLambda)
{
    Frustum splitted;
    splitted.leftBottomNear  = leftBottomNear*(1.0-nearLambda) + leftBottomFar*nearLambda;
	splitted.rightBottomNear = rightBottomNear*(1.0-nearLambda) + rightBottomFar*nearLambda;
	splitted.leftTopNear     = leftTopNear*(1.0-nearLambda) + leftTopFar*nearLambda;
	splitted.rightTopNear    = rightTopNear*(1.0-nearLambda) + rightTopFar*nearLambda;

    splitted.leftBottomFar  = leftBottomNear*(1.0-farLambda) + leftBottomFar*farLambda;
	splitted.rightBottomFar = rightBottomNear*(1.0-farLambda) + rightBottomFar*farLambda;
	splitted.leftTopFar     = leftTopNear*(1.0-farLambda) + leftTopFar*farLambda;
	splitted.rightTopFar    = rightTopNear*(1.0-farLambda) + rightTopFar*farLambda;
    splitted.computeBoundingBox();
    splitted.computePlanes();
    return splitted;

}

Frustum Frustum::transformedWith(const Matrix4x4 &transform)
{
    Frustum transformed;
    transformed.leftBottomNear  = (transform*Vector4(leftBottomNear, 1)).xyz();
	transformed.rightBottomNear = (transform*Vector4(rightBottomNear, 1)).xyz();
	transformed.leftTopNear     = (transform*Vector4(leftTopNear, 1)).xyz();
	transformed.rightTopNear    = (transform*Vector4(rightTopNear, 1)).xyz();
	transformed.leftBottomFar   = (transform*Vector4(leftBottomFar, 1)).xyz();
	transformed.rightBottomFar  = (transform*Vector4(rightBottomFar, 1)).xyz();
	transformed.leftTopFar      = (transform*Vector4(leftTopFar, 1)).xyz();
	transformed.rightTopFar     = (transform*Vector4(rightTopFar, 1)).xyz();
    transformed.computePlanes();
    transformed.computeBoundingBox();
    return transformed;
}

Frustum Frustum::transformedWith(const RigidTransform &transform)
{
    Frustum transformed;
    transformed.leftBottomNear  = transform.transformPosition(leftBottomNear);
	transformed.rightBottomNear = transform.transformPosition(rightBottomNear);
	transformed.leftTopNear     = transform.transformPosition(leftTopNear);
	transformed.rightTopNear    = transform.transformPosition(rightTopNear);
	transformed.leftBottomFar   = transform.transformPosition(leftBottomFar);
	transformed.rightBottomFar  = transform.transformPosition(rightBottomFar);
	transformed.leftTopFar      = transform.transformPosition(leftTopFar);
	transformed.rightTopFar     = transform.transformPosition(rightTopFar);
    transformed.computePlanes();
    transformed.computeBoundingBox();
    return transformed;
}
} // End of namespace UPhysics
