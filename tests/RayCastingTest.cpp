#include "uphysics/CollisionShape.hpp"
#include "uphysics/RigidBody.hpp"
#include "uphysics/PhysicsWorld.hpp"
#include "uphysics/Frustum.hpp"
#include <stdio.h>

using namespace UPhysics;

int screenWidth = 640;
int screenHeight = 480;

#ifdef WIN32
#define PACKED
#else
#define PACKED __attribute__((packed))
#endif 

struct TgaHeader
{
    uint8_t idLength;
    uint8_t colorMapType;
    uint8_t imageType;
    
    // Color map spec
    uint16_t firstColorMapEntry;
    uint16_t colorMapLength;
    uint8_t colorMapEntrySize;

    // Image spec
    uint16_t xOrigin;
    uint16_t yOrigin;
    uint16_t width;
    uint16_t height;
    uint8_t pixelDepth;
    uint8_t imageDescriptor;
} PACKED;

void renderShapeAndSaveImage(const char *filename, const CollisionShapePtr &collisionShape, Vector3 cameraCenter)
{
    std::unique_ptr<uint32_t[]> imageBuffer(new uint32_t[screenWidth * screenHeight]);
    
    auto destRow = imageBuffer.get();

    RigidTransform transform = {};
    transform.translation = cameraCenter;

    Frustum frustum;
    frustum.makePerspective(60.0f, float(screenWidth)/float(screenHeight), 0.1, 1000.0f);
    frustum = frustum.transformedWith(transform);

    for(int j = 0; j < screenHeight; ++j )
    {
        float v = float(j) / screenHeight;

        for(int i = 0; i < screenWidth; ++i )
        {
            float u = float(i) / screenWidth;
            
            auto ray = frustum.rayForNormalizedPoint(Vector2(u, v));
            auto rayCastResult = collisionShape->rayCast(ray);

            auto destPixel = destRow + i;
            if(rayCastResult)
            {
                auto N = rayCastResult->normal.safeNormalized();
                auto VdotN = std::abs(ray.direction.dot(N));
                uint8_t gray = uint8_t(VdotN*255);

                //printf("rayCastResult %f (%f %f %f) %f:%d\n", rayCastResult->distance, rayCastResult->normal.x, rayCastResult->normal.y, rayCastResult->normal.z, VdotN, gray);
                *destPixel = (0xFF<<24) | (gray << 16) | (gray << 8) | gray;
            }
            else
            {
                *destPixel = 0xFF202080;
            }
        }

        destRow += screenWidth;
    }

    TgaHeader header = {};
    header.imageType = 2;
    header.width = screenWidth;
    header.height = screenHeight;
    header.pixelDepth = 32;

    FILE *f = fopen(filename, "wb");
    if(!f)
    {
        fprintf(stderr, "Failed to open file: %s\n", filename);
        return;
    }

    fwrite(&header, sizeof(header), 1, f);
    fwrite(imageBuffer.get(), screenWidth*screenHeight*4, 1, f);
    fclose(f);

}

void renderWorldAndSaveImage(const char *filename, const PhysicsWorldPtr &physicsWorld, Vector3 cameraCenter)
{
    std::unique_ptr<uint32_t[]> imageBuffer(new uint32_t[screenWidth * screenHeight]);
    
    auto destRow = imageBuffer.get();

    RigidTransform transform = {};
    transform.translation = cameraCenter;

    Frustum frustum;
    frustum.makePerspective(60.0f, float(screenWidth)/float(screenHeight), 0.1, 1000.0f);
    frustum = frustum.transformedWith(transform);

    for(int j = 0; j < screenHeight; ++j )
    {
        float v = float(j) / screenHeight;

        for(int i = 0; i < screenWidth; ++i )
        {
            float u = float(i) / screenWidth;
            
            auto ray = frustum.rayForNormalizedPoint(Vector2(u, v));
            auto rayCastResult = physicsWorld->rayCast(ray);

            auto destPixel = destRow + i;
            if(rayCastResult)
            {
                auto N = rayCastResult->normal.safeNormalized();
                auto VdotN = std::abs(ray.direction.dot(N));
                uint8_t gray = uint8_t(VdotN*255);

                //printf("rayCastResult %f (%f %f %f) %f:%d\n", rayCastResult->distance, rayCastResult->normal.x, rayCastResult->normal.y, rayCastResult->normal.z, VdotN, gray);
                *destPixel = (0xFF<<24) | (gray << 16) | (gray << 8) | gray;
            }
            else
            {
                *destPixel = 0xFF2020FF;
            }
        }

        destRow += screenWidth;
    }

    TgaHeader header = {};
    header.imageType = 2;
    header.width = screenWidth;
    header.height = screenHeight;
    header.pixelDepth = 32;

    FILE *f = fopen(filename, "wb");
    if(!f)
    {
        fprintf(stderr, "Failed to open file: %s\n", filename);
        return;
    }

    fwrite(&header, sizeof(header), 1, f);
    fwrite(imageBuffer.get(), screenWidth*screenHeight*4, 1, f);
    fclose(f);

}

void testBoxShape()
{
    auto boxCollisionShape = std::make_shared<BoxCollisionShape> ();
    boxCollisionShape->setHalfExtent(Vector3(0.5, 0.5, 0.5));
    renderShapeAndSaveImage("testBoxShape.tga", boxCollisionShape, Vector3(0, 1, 3));
};

void testSphereShape()
{
    auto sphereCollisionShape = std::make_shared<SphereCollisionShape> ();
    sphereCollisionShape->setRadius(0.5);
    renderShapeAndSaveImage("testSphereShape.tga", sphereCollisionShape, Vector3(0, 1, 3));
};

void testFallingBoxWithCompoundFloor()
{
    auto physicsWorld = std::make_shared<PhysicsWorld> ();

    // Create the floor
    auto floorCollisionShape = std::make_shared<BoxCollisionShape> ();
    floorCollisionShape->setHalfExtent(Vector3(5, 0.25, 5));
    
    auto floorCompoundShape = std::make_shared<CompoundCollisionShape> ();
    floorCompoundShape->addElement(RigidTransform(), floorCollisionShape);

    auto floorCollisionObject = std::make_shared<CollisionObject> ();
    floorCollisionObject->collisionShape = floorCompoundShape;
    physicsWorld->addCollisionObject(floorCollisionObject);

    //auto floorBBox = floorCollisionObject->getWorldBoundingBox();

    // Create the fallling box
    auto boxCollisionShape = std::make_shared<BoxCollisionShape> ();
    boxCollisionShape->setHalfExtent(Vector3(0.5, 0.5, 0.5));

    auto rigidBody = std::make_shared<RigidBody> ();
    rigidBody->collisionShape = boxCollisionShape;
    rigidBody->setMass(1.0);
    rigidBody->computeMassDistribution();
    rigidBody->setPosition(Vector3(0, 2, 0));
    physicsWorld->addCollisionObject(rigidBody);

    renderWorldAndSaveImage("testFallingBoxWithCompoundFloor.tga", physicsWorld, Vector3(0, 1, 5));
}


int main()
{
    testBoxShape();
    testSphereShape();
    testFallingBoxWithCompoundFloor();
    return 0;
}