#include "fracture_allocator.inl"
[[vk::push_constant]] FracturePublicationPushConstants P;
[shader("compute")]
[numthreads(1,1,1)]
void entry_fracture_layout(uint3 tid : SV_DispatchThreadID)
{
  RigidBody* bodies=(RigidBody*)P.bodies_addr;
  Aabb* primitives=(Aabb*)P.primitives_addr;
  daxa_BlasInstanceData* instances=(daxa_BlasInstanceData*)P.instances_addr;
  uint offset=0u;
  for (uint id=0u;id<P.body_count;++id)
  {
    RigidBody* body=&bodies[id];
    body->primitive_offset=offset;
    if (body->shape_index==0u) {
      primitives[offset].minimum=body->minimum;
      primitives[offset].maximum=body->maximum;
    }
    instances[id].transform=body->get_instance_transform();
    offset+=body->primitive_count;
  }
}
