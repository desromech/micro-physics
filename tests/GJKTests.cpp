#include "uphysics/GJK.hpp"

#include <stdlib.h>
#include <stdio.h>

#define TestAssert(expr) \
if(!(expr)) { \
    fprintf(stderr, "%s.%d: Test assertion failure: %s\n", __FILE__, __LINE__, #expr); \
    abort(); \
}

using namespace UPhysics;

void testEmpty()
{
    GJKVoronoiSimplexSolver simplex;
    TestAssert(simplex.pointCount == 0);
    simplex.reduce();
    TestAssert(simplex.pointCount == 0);
}

void testSinglePoint()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(1, 0, 0));
    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(1, 0, 0));
    TestAssert(!simplex.containsOrigin());
}

void testSinglePoint2()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(0, 0, 0));
    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, 0, 0));
    TestAssert(simplex.containsOrigin());
}

void testLineFirst()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(1, 0, 0));
    simplex.insertPoint(Vector3(2, 0, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 1);
    TestAssert(simplex.barycentricCoordinates[1] == 0);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

	simplex.reduce();
    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 1);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());
}

void testLineSecond()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(-2, 0, 0));
    simplex.insertPoint(Vector3(-1, 0, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(-1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0);
    TestAssert(simplex.barycentricCoordinates[1] == 1);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

	simplex.reduce();
    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(-1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 1);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());
}

void testLineInside()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(-1, 0, 0));
    simplex.insertPoint(Vector3(1, 0, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(simplex.containsOrigin());
}

void testLineInside2()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(1, 0, 0));
    simplex.insertPoint(Vector3(-1, 0, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(simplex.containsOrigin());
}

void testLineMiddle()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(-1, 1, 0));
    simplex.insertPoint(Vector3(1, 1, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, 1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

    simplex.reduce();
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);

}

void testLineMiddle2()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(1, 1, 0));
    simplex.insertPoint(Vector3(-1, 1, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, 1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

    simplex.reduce();
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);

}

void testTriangleA()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(1, 0, 0));
    simplex.insertPoint(Vector3(3, 0, 0));
    simplex.insertPoint(Vector3(2, 1, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 1);
    TestAssert(simplex.barycentricCoordinates[1] == 0);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

    simplex.reduce();
    TestAssert(simplex.pointCount == 1);
    TestAssert(simplex.points[0] == Vector3(1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 1);
}

void testTriangleB()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(3, 0, 0));
    simplex.insertPoint(Vector3(1, 0, 0));
    simplex.insertPoint(Vector3(2, 1, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0);
    TestAssert(simplex.barycentricCoordinates[1] == 1);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

    simplex.reduce();
    TestAssert(simplex.pointCount == 1);
    TestAssert(simplex.points[0] == Vector3(1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 1);
}

void testTriangleC()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(3, 0, 0));
    simplex.insertPoint(Vector3(2, 1, 0));
    simplex.insertPoint(Vector3(1, 0, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0);
    TestAssert(simplex.barycentricCoordinates[1] == 0);
    TestAssert(simplex.barycentricCoordinates[2] == 1);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

    simplex.reduce();
    TestAssert(simplex.pointCount == 1);
    TestAssert(simplex.points[0] == Vector3(1, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 1);
}

void testTriangleAB()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(-1, 1, 0));
    simplex.insertPoint(Vector3(1, 1, 0));
    simplex.insertPoint(Vector3(0, 2, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, 1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
    TestAssert(simplex.barycentricCoordinates[2] == 0);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

    simplex.reduce();
    TestAssert(simplex.pointCount == 2);
    TestAssert(simplex.points[0] == Vector3(-1, 1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.points[1] == Vector3(1, 1, 0));
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
}

void testTriangleBC()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(0, 2, 0));
    simplex.insertPoint(Vector3(-1, 1, 0));
    simplex.insertPoint(Vector3(1, 1, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, 1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0);
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
    TestAssert(simplex.barycentricCoordinates[2] == 0.5);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

    simplex.reduce();
    TestAssert(simplex.pointCount == 2);
    TestAssert(simplex.points[0] == Vector3(-1, 1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.points[1] == Vector3(1, 1, 0));
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
}

void testTriangleCA()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(1, 1, 0));
    simplex.insertPoint(Vector3(0, 2, 0));
    simplex.insertPoint(Vector3(-1, 1, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, 1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.barycentricCoordinates[1] == 0);
    TestAssert(simplex.barycentricCoordinates[2] == 0.5);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

    simplex.reduce();
    TestAssert(simplex.pointCount == 2);
    TestAssert(simplex.points[0] == Vector3(1, 1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.5);
    TestAssert(simplex.points[1] == Vector3(-1, 1, 0));
    TestAssert(simplex.barycentricCoordinates[1] == 0.5);
}

void testTriangleInside()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(-1, -1, 0));
    simplex.insertPoint(Vector3(1, -1, 0));
    simplex.insertPoint(Vector3(0, 1, 0));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, 0, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.25);
    TestAssert(simplex.barycentricCoordinates[1] == 0.25);
    TestAssert(simplex.barycentricCoordinates[2] == 0.5);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(simplex.containsOrigin());
}

void testTriangleMiddle()
{
    GJKVoronoiSimplexSolver simplex;
    simplex.insertPoint(Vector3(-1, -1, 1));
    simplex.insertPoint(Vector3(1, -1, 1));
    simplex.insertPoint(Vector3(0, -1, -1));

    TestAssert(simplex.computeClosesPointToOrigin() == Vector3(0, -1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.25);
    TestAssert(simplex.barycentricCoordinates[1] == 0.25);
    TestAssert(simplex.barycentricCoordinates[2] == 0.5);
    TestAssert(simplex.barycentricCoordinates[3] == 0);
    TestAssert(!simplex.containsOrigin());

    simplex.reduce();
    TestAssert(simplex.pointCount == 3);
    TestAssert(simplex.closestPointToOrigin == Vector3(0, 1, 0));
    TestAssert(simplex.barycentricCoordinates[0] == 0.25);
    TestAssert(simplex.barycentricCoordinates[1] == 0.25);
    TestAssert(simplex.barycentricCoordinates[2] == 0.5);
    TestAssert(!simplex.containsOrigin());
}

int main()
{
    testEmpty();

    testSinglePoint();
    testSinglePoint2();
    
    testLineFirst();
    testLineSecond();
    testLineInside();
    testLineInside2();
    testLineMiddle();
    testLineMiddle2();
    
    testTriangleA();
    testTriangleB();
    testTriangleC();
    
    testTriangleAB();
    testTriangleBC();
    testTriangleCA();

    testTriangleInside();
    return 0;
}