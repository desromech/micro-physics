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
int main()
{
    testQuaternionMultiplication();
    return 0;
}