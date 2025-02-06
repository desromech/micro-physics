#include "uphysics/CollisionObject.hpp"
#include <atomic>

namespace UPhysics
{

static std::atomic<uint32_t> monotonicIDCounter;
CollisionObject::CollisionObject()
{
    monotonicID = monotonicIDCounter.fetch_add(1);
}

CollisionObject::~CollisionObject()
{
}

} // End of namespace UPhysics