#include "uphysics/Quaternion.hpp"

#include <stdlib.h>
#include <stdio.h>

#define TestAssert(expr) \
if(!(expr)) { \
    fprintf(stderr, "%s.%d: Test assertion failure: %s\n", __FILE__, __LINE__, #expr); \
    abort(); \
}

using namespace UPhysics;

void testQuaternionMultiplication()
{
    TestAssert((Quaternion(2, 3, 4, 1) * Quaternion(31, 41, 51, 20)) == Quaternion(60, 123, 120, -369));
}

void testQuaternionLogarithm()
{
    TestAssert(Quaternion(0.1, 0, 0, 0).exp().ln().closeTo(Quaternion(0.1, 0, 0, 0)))
    TestAssert(Quaternion(0.0, 0.1, 0, 0).exp().ln().closeTo(Quaternion(0.0, 0.1, 0, 0)))
    TestAssert(Quaternion(0.0, 0, 0.1, 0).exp().ln().closeTo(Quaternion(0.0, 0, 0.1, 0)))
}

int main()
{
    testQuaternionMultiplication();
    testQuaternionLogarithm();
    return 0;
}