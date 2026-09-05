#pragma once
#include "shared.inl"

bool rb_is_awake_dynamic(RigidBody* r)
{
  return ((r->flags & RigidBodyFlag::DYNAMIC) != RigidBodyFlag::NONE) &&
         ((r->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE);
}
