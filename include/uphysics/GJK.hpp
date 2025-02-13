#ifndef UPHYSICS_GJK_HPP
#define UPHYSICS_GJK_HPP

#include "Vector3.hpp"
#include "Math.hpp"
#include "Ray.hpp"
#include "ContactPoint.hpp"
#include <assert.h>
#include <stdint.h>
#include <optional>

namespace UPhysics
{

class GJKVoronoiSimplexSolver
{
public:
    static const size_t MaxPoints = 4;

    GJKVoronoiSimplexSolver()
    {
        for(size_t i = 0; i < MaxPoints; ++i)
        {
            points[i] = Vector3::zeros();
            firstPoints[i] = Vector3::zeros();
            secondPoints[i] = Vector3::zeros();
            barycentricCoordinates[i] = 0.0f;
            usedPoints[i] = false;
            closestPointToOrigin = Vector3::zeros();
        }
    }
    void reduce()
    {
        uint32_t destIndex = 0;
        for(uint32_t i = 0; i < pointCount; ++i)
        {
            if(usedPoints[i])
            {
                usedPoints[destIndex] = usedPoints[i];
                barycentricCoordinates[destIndex] = barycentricCoordinates[i];
                points[destIndex] = points[i];
                firstPoints[destIndex] = firstPoints[i];
                secondPoints[destIndex] = secondPoints[i];
                destIndex++;
            }
        }

        pointCount = destIndex;
    }

    void invalidateCache()
    {
        hasComputedClosest = false;
    }

    Vector3 computeClosesPointToOrigin()
    {
        if(hasComputedClosest)
            return closestPointToOrigin;

        switch(pointCount)
        {
        case 0:
            hasComputedClosest = true;
            closestPointToOrigin = Vector3(0, 0, 0);
            hasComputedClosest = true;
        case 1:
            closestPointToOrigin = points[0];
            usedPoints[0] = true;
            usedPoints[1] = false;
            usedPoints[2] = false;
            usedPoints[3] = false;
            barycentricCoordinates[0] = 1;
            barycentricCoordinates[1] = 0;
            barycentricCoordinates[2] = 0;
            barycentricCoordinates[3] = 0;
            hasComputedClosest = true;
            break;
        case 2:
            lineCase();
            break;
        case 3:
            triangleCase();
            break;
        case 4:
            tetrahedronCase();
            break;
        default:
            assert(false && "Imposible case.");
        }

        return closestPointToOrigin;
    }

    void lineCase()
    {
        auto from = points[0];
        auto to = points[1];

        auto delta = to - from;
        auto lambda = (Vector3(0,0,0) - from).dot(delta);
        if(lambda <= 0)
        {
            barycentricCoordinates[0] = 1;
            barycentricCoordinates[1] = 0;
            barycentricCoordinates[2] = 0;
            barycentricCoordinates[3] = 0;
            usedPoints[0] = true;
            usedPoints[1] = false;
            usedPoints[2] = false;
            usedPoints[3] = false;
            closestPointToOrigin = from;
            hasComputedClosest = true;
            return;
        }

        lambda = lambda / delta.length2();
        if(lambda >= 1)
        {
            barycentricCoordinates[0] = 0;
            barycentricCoordinates[1] = 1;
            barycentricCoordinates[2] = 0;
            barycentricCoordinates[3] = 0;
            usedPoints[0] = false;
            usedPoints[1] = true;
            usedPoints[2] = false;
            usedPoints[3] = false;
            closestPointToOrigin = to;
            hasComputedClosest = true;
            return;
        }

        closestPointToOrigin = from + (delta * lambda);
        barycentricCoordinates[0] = 1 - lambda;
        barycentricCoordinates[1] = lambda;
        barycentricCoordinates[2] = 0;
        barycentricCoordinates[3] = 0;
        usedPoints[0] = true;
        usedPoints[1] = true;
        usedPoints[2] = false;
        usedPoints[3] = false;
        hasComputedClosest = true;
    }

    void triangleCase()
    {
        //Algorithm from 'Real Time Collision detection' by Ericson. ClosestPtPointTriangle.
        auto a = points[0];
        auto b = points[1];
        auto c = points[2];
        auto p = Vector3(0,0,0);

        auto ab = b - a;
        auto ac = c - a;
        auto ap = p - a ;
        auto d1 = ab.dot(ap);
        auto d2 = ac.dot(ap);

        // A vertex region
        if(d1 <= 0.0f && d2 <= 0.0f)
        {
            barycentricCoordinates[0] = 1;
            barycentricCoordinates[1] = 0;
            barycentricCoordinates[2] = 0;
            barycentricCoordinates[3] = 0;
            usedPoints[0] = true;
            usedPoints[1] = false;
            usedPoints[2] = false;
            usedPoints[3] = false;
            closestPointToOrigin = a;
            hasComputedClosest = true;
            return;
        }

        // B vertex region
        auto bp = p - b;
        auto d3 = ab.dot(bp);
        auto d4 = ac.dot(bp);
        if(d3 >= 0.0 && d4 <= d3)
        {
            barycentricCoordinates[0] = 0;
            barycentricCoordinates[1] = 1;
            barycentricCoordinates[2] = 0;
            barycentricCoordinates[3] = 0;
            usedPoints[0] = false;
            usedPoints[1] = true;
            usedPoints[2] = false;
            usedPoints[3] = false;
            closestPointToOrigin = b;
            hasComputedClosest = true;
            return;
        }

        // Edge region AB
        auto vc = d1*d4 - d3*d2;
        if(vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        {
            auto v = d1/(d1 - d3);

            barycentricCoordinates[0] = 1 - v;
            barycentricCoordinates[1] = v;
            barycentricCoordinates[2] = 0;
            barycentricCoordinates[3] = 0;
            usedPoints[0] = true;
            usedPoints[1] = true;
            usedPoints[2] = false;
            usedPoints[3] = false;
            closestPointToOrigin = a + ab*v;
            hasComputedClosest = true;
            return;
        }

        // Vertex region C
        auto cp = p - c;
        float d5 = ab.dot(cp);
        float d6 = ac.dot(cp);
        if(d6 >= 0.0 && d5 <= d6)
        {
            barycentricCoordinates[0] = 0;
            barycentricCoordinates[1] = 0;
            barycentricCoordinates[2] = 1;
            barycentricCoordinates[3] = 0;
            usedPoints[0] = false;
            usedPoints[1] = false;
            usedPoints[2] = true;
            usedPoints[3] = false;
            closestPointToOrigin = c;
            hasComputedClosest = true;
            return;
        }
        
        auto vb = d5*d2 - d1*d6;
        if(vb <= 0.0f && d2 >= 0.0 && d6 <= 0.0f)
        {
            auto w = d2/(d2- d6);
            barycentricCoordinates[0] = 1 -w;
            barycentricCoordinates[1] = 0;
            barycentricCoordinates[2] = w;
            barycentricCoordinates[3] = 0;
            usedPoints[0] = true;
            usedPoints[1] = false;
            usedPoints[2] = true;
            usedPoints[3] = false;
            closestPointToOrigin = a + ac*w;
            hasComputedClosest = true;
            return;
        }

        float va = d3*d6 - d5*d4;
        if(va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
        {
            float w = (d4 - d3)/ ((d4 - d3) + (d5 - d6));
            barycentricCoordinates[0] = 0;
            barycentricCoordinates[1] = 1 - w;
            barycentricCoordinates[2] = w;
            barycentricCoordinates[3] = 0;
            usedPoints[0] = false;
            usedPoints[1] = true;
            usedPoints[2] = true;
            usedPoints[3] = false;
            closestPointToOrigin = b + (c - b)*w;
            hasComputedClosest = true;
            return;
        }

        // P inside face region
        float denom = 1.0f / (va + vb + vc);
        float u = va*denom;
        float v = vb*denom;
        float w = vc*denom;
        barycentricCoordinates[0] = u;
        barycentricCoordinates[1] = v;
        barycentricCoordinates[2] = w;
        barycentricCoordinates[3] = 0;
        usedPoints[0] = true;
        usedPoints[1] = true;
        usedPoints[2] = true;
        usedPoints[3] = false;
        closestPointToOrigin = a + ab*v + ac*w;
        hasComputedClosest = true;
        return;
    }

    float computeSideOfPointInPlane(Vector3 p, Vector3 p4, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        auto u = p2 - p1;
        auto v = p3 - p1;
        auto n = u.cross(v);
        auto d = p1.dot(n);
        auto signP = sign((p.dot(n)) - d);
        auto signP4 = sign ((p4.dot(n)) - d);
        return signP*signP4;
    }
    void tetrahedronCase()
    {
        auto a = points[0]; auto la = firstPoints[0]; auto ra = secondPoints[0];
        auto b = points[1]; auto lb = firstPoints[1]; auto rb = secondPoints[1];
        auto c = points[2]; auto lc = firstPoints[2]; auto rc = secondPoints[2];
        auto d = points[3]; auto ld = firstPoints[3]; auto rd = secondPoints[3];
        auto p = Vector3(0, 0, 0);

        auto abcSide = computeSideOfPointInPlane(p, d, a, b, c);
        auto abdSide = computeSideOfPointInPlane(p, c, a, b, d);
        auto acdSide = computeSideOfPointInPlane(p, b, a, c, d);
        auto bcdSide = computeSideOfPointInPlane(p, a, b, c, d);

        // Is Inside?
        if (abcSide >= 0 && abdSide >= 0 && acdSide >= 0 && bcdSide >= 0)
        {
            closestPointToOrigin = Vector3(0, 0, 0);
            // FIXME: Compute this coordinate properly'.
            barycentricCoordinates[0] = 1;
            barycentricCoordinates[1] = 1;
            barycentricCoordinates[2] = 1;
            barycentricCoordinates[3] = 1;
            usedPoints[0] = true;
            usedPoints[1] = true;
            usedPoints[2] = true;
            usedPoints[3] = true;
            hasComputedClosest = true;
            return;
        }

        auto bestResult = INFINITY;
        if(abcSide < 0)
        {
            GJKVoronoiSimplexSolver subsimplex;
            subsimplex.insertPointOnObjects(a, la, ra);
            subsimplex.insertPointOnObjects(b, lb, rb);
            subsimplex.insertPointOnObjects(c, lc, rc);
            Vector3 closestPoint = subsimplex.computeClosesPointToOrigin();
            float closestLength = closestPoint.length();
            if(closestLength < bestResult)
            {
                bestResult = closestLength;
                *this = subsimplex;
            }
        }

        if(abdSide < 0)
        {
            GJKVoronoiSimplexSolver subsimplex;
            subsimplex.insertPointOnObjects(a, la, ra);
            subsimplex.insertPointOnObjects(b, lb, rb);
            subsimplex.insertPointOnObjects(d, ld, rd);
            Vector3 closestPoint = subsimplex.computeClosesPointToOrigin();
            float closestLength = closestPoint.length();
            if(closestLength < bestResult)
            {
                bestResult = closestLength;
                *this = subsimplex;
            }
        }

        if(acdSide < 0)
        {
            GJKVoronoiSimplexSolver subsimplex;
            subsimplex.insertPointOnObjects(a, la, ra);
            subsimplex.insertPointOnObjects(c, lc, rc);
            subsimplex.insertPointOnObjects(d, ld, rd);
            Vector3 closestPoint = subsimplex.computeClosesPointToOrigin();
            float closestLength = closestPoint.length();
            if(closestLength < bestResult)
            {
                bestResult = closestLength;
                *this = subsimplex;
            }
        }

        if(bcdSide < 0)
        {
            GJKVoronoiSimplexSolver subsimplex;
            subsimplex.insertPointOnObjects(b, lb, rb);
            subsimplex.insertPointOnObjects(c, lc, rc);
            subsimplex.insertPointOnObjects(d, ld, rd);
            Vector3 closestPoint = subsimplex.computeClosesPointToOrigin();
            float closestLength = closestPoint.length();
            if(closestLength < bestResult)
            {
                bestResult = closestLength;
                *this = subsimplex;
            }
        }
    }

    template<typename FT>
    void transformPointsWith(const FT &transform)
    {
        for(size_t i = 0; i < pointCount; ++i)
        {
            points[i] = transform(points[i]);
            firstPoints[i] = transform(firstPoints[i]);
            secondPoints[i] = transform(secondPoints[i]);
        }
        invalidateCache();
    }

    bool containsOrigin()
    {
        return pointCount > 0 && computeClosesPointToOrigin().closeTo(Vector3(0, 0, 0));
    }

    bool containsPoint(const Vector3 &pointToTest)
    {
        for(auto &p : points)
        {
            if(p.closeTo(pointToTest))
                return true;
        }

        return false;
    }

    void insertPoint(Vector3 point)
    {
        insertPointOnObjects(point, point, point);
    }

    void insertPointOnObjects(Vector3 point, Vector3 firstObjectPoint, Vector3 secondObjectPoint)
    {
        assert(pointCount < MaxPoints);
        points[pointCount] = point;
        firstPoints[pointCount] = firstObjectPoint;
        secondPoints[pointCount] = secondObjectPoint;
        ++pointCount; 
        invalidateCache();
    }

    Vector3 computeClosesPointToOriginInFirstObject()
    {
        if(!hasComputedClosest)
            computeClosesPointToOrigin();
        
        auto result = Vector3::zeros();
        auto barycentricSum = 0.0f;
        for(size_t i = 0; i < pointCount; ++i)
        {
            result += firstPoints[i]*barycentricCoordinates[i];
            barycentricSum += barycentricCoordinates[i];
        }

        return result / Vector3(barycentricSum);
    }

    Vector3 computeClosesPointToOriginInSecondObject()
    {
        if(!hasComputedClosest)
            computeClosesPointToOrigin();
        
        auto result = Vector3::zeros();
        auto barycentricSum = 0.0f;
        for(size_t i = 0; i < pointCount; ++i)
        {
            result += secondPoints[i]*barycentricCoordinates[i];
            barycentricSum += barycentricCoordinates[i];
        }

        return result / Vector3(barycentricSum);
    }

    size_t pointCount = 0;
    Vector3 points[MaxPoints];
    Vector3 firstPoints[MaxPoints];
    Vector3 secondPoints[MaxPoints];
    float barycentricCoordinates[MaxPoints];
    bool usedPoints[MaxPoints];
    Vector3 closestPointToOrigin;
    bool hasComputedClosest = false;
};

template<typename FirstSupportFunction, typename SecondSupportFunction>
GJKVoronoiSimplexSolver computeGJKSimplexFor(FirstSupportFunction firstObjectSupport, SecondSupportFunction secondObjectSupport, Vector3 startingDirectionHint = Vector3(1, 0, 0))
{
    static const int MaxNumberOfIterations = 32;
    static const float Epsilon = 1e-5;

    if(startingDirectionHint.closeTo(Vector3::zeros()))
        startingDirectionHint = Vector3(1, 0, 0);


    GJKVoronoiSimplexSolver simplex;
    auto nextDirection = startingDirectionHint;
    Vector3 firstSupportPoint = firstObjectSupport(nextDirection);
    Vector3 secondSupportPoint = secondObjectSupport(-nextDirection);
    auto lastPoint = firstSupportPoint - secondSupportPoint;
    simplex.insertPointOnObjects(lastPoint, firstSupportPoint, secondSupportPoint);
    nextDirection = -lastPoint;

    int remainingIterations = MaxNumberOfIterations;
    while (remainingIterations > 0)
    {
        firstSupportPoint = firstObjectSupport(nextDirection);
        secondSupportPoint = secondObjectSupport(-nextDirection);
        auto nextPoint = firstSupportPoint - secondSupportPoint;

        // Are we getting closer to the origin?
		auto delta = nextPoint - lastPoint;
		if (delta.dot(nextDirection) <= Epsilon)
            return simplex;

        simplex.insertPointOnObjects(nextPoint, firstSupportPoint, secondSupportPoint);
        lastPoint = nextPoint;

        // Do we contain the origin?
        if (simplex.containsOrigin()) {
            return simplex;
        }

        // Advance in direction to the origin.
        nextDirection = -simplex.computeClosesPointToOrigin();

        // Reduce the simplex.
        simplex.reduce();
        --remainingIterations;
    }

    return simplex;

}

template<typename FirstSupportFunction, typename SecondSupportFunction>
ContactPointPtr samplePenetrationDistanceAndNormalForSupport(const FirstSupportFunction &firstSupport, const SecondSupportFunction &secondSupport, Vector3 startingDirectionHint = Vector3(1, 0, 0))
{
    static const Vector3 RandomSamplingDistribution[] = {
        {-0.7071067811865476, 0.0, -0.7071067811865476},
        {-0.5773502691896258, -0.5773502691896258, -0.5773502691896258},
        {-0.7071067811865476, -0.7071067811865476, 0.0},
        {-1.0, 0.0, 0.0},
        {-0.5773502691896258, -0.5773502691896258, 0.5773502691896258},
        {-0.7071067811865476, 0.0, 0.7071067811865476},
        {-0.5773502691896258, 0.5773502691896258, 0.5773502691896258},
        {-0.7071067811865476, 0.7071067811865476, 0.0},
        {-0.5773502691896258, 0.5773502691896258, -0.5773502691896258},
        {0.7071067811865476, 0.0, -0.7071067811865476},
        {0.5773502691896258, 0.5773502691896258, -0.5773502691896258},
        {0.7071067811865476, 0.7071067811865476, 0.0},
        {1.0, 0.0, 0.0},
        {0.5773502691896258, 0.5773502691896258, 0.5773502691896258},
        {0.7071067811865476, 0.0, 0.7071067811865476},
        {0.5773502691896258, -0.5773502691896258, 0.5773502691896258},
        {0.7071067811865476, -0.7071067811865476, 0.0},
        {0.5773502691896258, -0.5773502691896258, -0.5773502691896258},
        {0.0, -0.7071067811865476, -0.7071067811865476},
        {0.0, -1.0, 0.0},
        {0.0, -0.7071067811865476, 0.7071067811865476},
        {0.0, 0.7071067811865476, -0.7071067811865476},
        {0.0, 1.0, 0.0},
        {0.0, 0.7071067811865476, 0.7071067811865476},
        {0.0, 0.0, -1.0},
        {0.0, 0.0, 1.0},
    };

	float bestDistance = INFINITY;
	Vector3 bestNormal = Vector3(0, 0, 0);
	Vector3 bestFirstPoint = Vector3(0, 0, 0);
	Vector3 bestSecondPoint = Vector3(0, 0, 0);

    auto && sampleBlock = [&](const Vector3 sampleVector) {
		Vector3 firstPoint = firstSupport(-sampleVector);
		Vector3 secondPoint = secondSupport(sampleVector);
		
		auto supportVector = secondPoint - firstPoint;
		auto delta = supportVector.dot(sampleVector);
		if (delta < bestDistance) {
			bestDistance = delta;
			bestNormal = sampleVector;
			bestFirstPoint = firstPoint;
			bestSecondPoint = secondPoint;
        }
    };

    sampleBlock(startingDirectionHint);
    sampleBlock(-startingDirectionHint);

    for(auto &direction : RandomSamplingDistribution)
        sampleBlock(direction);

    if (bestDistance < 0)
        return nullptr;

    assert(!isnan(bestDistance));
    assert(!bestNormal.hasNaN());
    assert(!bestFirstPoint.hasNaN());
    assert(!bestSecondPoint.hasNaN());

    auto result = std::make_shared<ContactPoint> ();
    result->penetrationDistance = bestDistance;
    result->normal = bestNormal;
    result->firstPoint = bestFirstPoint;
    result->secondPoint = bestSecondPoint;
    return result;
}
template<typename FirstSupportFunction, typename SecondSupportFunction>
ContactPointPtr samplePenetrationSupportContact(const FirstSupportFunction &firstSupport, const SecondSupportFunction &secondSupport, float margin, Vector3 startingDirectionHint = Vector3(1, 0, 0))
{
    auto distanceAndNormals = samplePenetrationDistanceAndNormalForSupport(firstSupport, secondSupport, startingDirectionHint);
    if(!distanceAndNormals)
        return nullptr;

    auto distance = distanceAndNormals->penetrationDistance;
    auto normal = distanceAndNormals->normal;
    
    float extraSeparation = 0.5;
    float distanceWithMargin = extraSeparation + distance + margin;
    auto displacement = normal * distanceWithMargin;
    auto && displacedFirstSupport = [&](const Vector3 &d){
        return firstSupport(d) + displacement;
    };

    GJKVoronoiSimplexSolver gjkSimplex = computeGJKSimplexFor(displacedFirstSupport, secondSupport, -normal);
    auto separationVector = gjkSimplex.computeClosesPointToOrigin();
    auto separationVectorLength = separationVector.length();

    auto correctedPenetrationDistance = distanceWithMargin - separationVectorLength;
    auto firstClosestPoint = gjkSimplex.computeClosesPointToOriginInFirstObject();
    auto secondClosestPoint = gjkSimplex.computeClosesPointToOriginInSecondObject();

    firstClosestPoint = firstClosestPoint - (normal * distanceWithMargin);

    auto result = std::make_shared<ContactPoint> ();
    result->requiredSeparation = margin;
    result->penetrationDistance = correctedPenetrationDistance;
    result->normal = normal;
    result->firstPoint = firstClosestPoint;
    result->secondPoint = secondClosestPoint;
    //result.computeWorldContactPointAndDistances();
    return result;

}

struct GJKRayCastResult
{
    float distance;        
    Vector3 normal;
};

template<typename SupportFunction>
std::optional<GJKRayCastResult> gjkRayCast(const Ray &ray, const SupportFunction &supportFunction)
{
    const int MaxNumberOfIterations = 32;
    const float Epsilon2 =1.0e-10;

    // Algorithm from 'Ray Casting against General Convex Objectswith Application to Continuous CollisionDetection' by G. Van Den Bergen.
	auto lambda = ray.tmin;
	auto lambdaMax = ray.tmax;
	auto s = ray.origin;
	auto r = ray.direction;
	auto x = s + (r * lambda);
	auto n = Vector3::zeros();

	// Code for testing the convex cast.
	Vector3 v = x - supportFunction(-r);
	auto simplex = GJKVoronoiSimplexSolver();
	auto remainingIterations = MaxNumberOfIterations;

    while(remainingIterations > 0 && v.length2() > Epsilon2)
    {
        if (lambda > lambdaMax)
            return std::nullopt;

        auto p = supportFunction(v);
        auto w = x - p;
        auto VdotW = v.dot(w);

        if(VdotW > 0)
        {
            auto VdotR = v.dot(r);
			if(VdotR >= -Epsilon2)
                return std::nullopt;

			lambda = lambda - (VdotW / VdotR);
			if(lambda > lambdaMax)
                return std::nullopt;

			auto oldX = x;
			x = s + (r * lambda);
			n = v;
			w = x - p;
			auto deltaX = x - oldX;
			simplex.transformPointsWith([&](const Vector3 &simplexPoint) {
                return simplexPoint + deltaX;
            });

			if (simplex.containsOrigin()) 
                return GJKRayCastResult{lambda, n};
			simplex.reduce();
        }
        if(!simplex.containsPoint(w))
            simplex.insertPoint(w);

        if(simplex.containsOrigin())
        {
            remainingIterations = 0;
        }
        else
        {
            v = simplex.computeClosesPointToOrigin();
            simplex.reduce();
            --remainingIterations;
        }
    }

    return GJKRayCastResult{lambda, n};
}

} // End of namespace UPhysics

#endif //UPHYSICS_GJK_HPP