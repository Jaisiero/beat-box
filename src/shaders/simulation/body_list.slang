#include "body_list.inl"
[[vk::push_constant]] BodyListPushConstants P;
[shader("compute")]
[numthreads(1,1,1)]
void entry_body_list(uint3 tid : SV_DispatchThreadID)
{
  RigidBody* bodies=(RigidBody*)P.bodies_addr;
  RigidBodyEntry* entries=(RigidBodyEntry*)P.entries_addr;
  ActiveRigidBody* active=(ActiveRigidBody*)P.active_addr;
  for (uint i=0u;i<P.count;++i) entries[i].index=MAX_U32;
  for (uint i=0u;i<P.count;++i)
    if (bodies[i].id < P.count) entries[bodies[i].id].index=i;
  uint count=0u;
  for (uint id=0u;id<P.count;++id)
  {
    uint row=entries[id].index;
    if (row < P.count && bodies[row].has_flag(RigidBodyFlag::DYNAMIC)) active[count++].rigid_body_id=id;
  }
}
