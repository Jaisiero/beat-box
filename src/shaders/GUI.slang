#include "shared.inl"

[[vk::push_constant]] GUIPushConstants UI;

static const daxa_f32vec3 POINT_COLOR = daxa_f32vec3(1.0, 0.0, 0.0);

// Per-vertex attributes to be assembled from bound vertex buffers.
struct AssembledVertex
{
  // float3 position : POSITION;
  uint index : INDEX;
};

// Output of the vertex shader, and input to the fragment shader.
struct CoarseVertex
{
  float3 color;
};

// Output of the fragment shader
struct Fragment
{
  float4 color;
};

// Vertex  Shader

struct VertexStageOutput
{
  CoarseVertex coarseVertex : CoarseVertex;
  float4 sv_position : SV_Position;
};

// hardcoded vertices for now
static const float3 vertices[3] = {
    float3(0.0, 0.5, 0.0),    // Vértice superior
    float3(0.5, -0.5, 0.0),   // Vértice inferior derecho
    float3(-0.5, -0.5, 0.0)   // Vértice inferior izquierdo
};
[shader("vertex")] VertexStageOutput entry_vertex(
    uint vertexID : SV_VertexID  // Obtenemos el índice del vértice actual
)
{
  VertexStageOutput output;

  // daxa_f32mat4x4 inv_view = Ptr<CameraView>(UI.task_head.camera).inv_view;
  // daxa_f32mat4x4 inv_proj = Ptr<CameraView>(UI.task_head.camera).inv_proj;

  // daxa_f32mat4x4 mvp = mul(inv_proj, inv_view);

  // Ptr<GUIVertex> gui_vertices = Ptr<GUIVertex>(UI.task_head.vertex_buffer);

  // float3 position = gui_vertices[assembledVertex.index].position;
  float3 position = vertices[vertexID];
  float3 color = POINT_COLOR;

  output.coarseVertex.color = color;
  // output.sv_position = mul(mvp, float4(position, 1.0));
  output.sv_position = float4(position, 1.0);

  return output;
}

    // Fragment Shader

[shader("fragment")] float4 entry_fragment(
  CoarseVertex coarseVertex : CoarseVertex) : SV_Target
{
  float3 color = coarseVertex.color;

  return float4(color, 1.0);
}