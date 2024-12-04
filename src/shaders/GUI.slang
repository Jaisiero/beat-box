#include "shared.inl"

[[vk::push_constant]] GUIPushConstants UI;
[[vk::push_constant]] GUILinePushConstants UIL;
[[vk::push_constant]] GUIAxesPushConstants UIA;

static const daxa_f32 POINT_MIN_SIZE = 2.0;
static const daxa_f32 POINT_MAX_SIZE = 5.0;

// Per-vertex attributes to be assembled from bound vertex buffers.
struct AssembledVertex
{
  // daxa_f32vec3 position : POSITION;
  uint index : INDEX;
};

// Output of the vertex shader, and input to the fragment shader.
struct CoarseVertex
{
  daxa_f32vec3 color;
};

// Output of the fragment shader
struct Fragment
{
  daxa_f32vec4 color;
};

struct VertexStageOutput
{
  CoarseVertex coarse_vertex : CoarseVertex;
  daxa_f32vec4 sv_position : SV_Position;
  float pointSize : SV_PointSize;
};

// Vertex  Shader
[shader("vertex")] VertexStageOutput entry_vertex(
    uint vertexID : SV_VertexID  // Current vertex index
)
{
  VertexStageOutput output;

  daxa_f32mat4x4 view = Ptr<CameraView>(UI.task_head.camera).view;
  daxa_f32mat4x4 proj = Ptr<CameraView>(UI.task_head.camera).proj;
  daxa_f32mat4x4 inv_view = Ptr<CameraView>(UI.task_head.camera).inv_view;

  daxa_f32mat4x4 mvp = mul(proj, view);

  Ptr<GUIVertex> gui_vertices = Ptr<GUIVertex>(UI.task_head.vertex_buffer);

  daxa_f32vec3 position = gui_vertices[vertexID].position;
  daxa_f32vec3 color = gui_vertices[vertexID].color;
  
  daxa_f32vec4 origin = mul(inv_view, daxa_f32vec4(0, 0, 0, 1));
  daxa_f32 distance = length(position - origin.xyz);

  output.coarse_vertex.color = color;
  output.sv_position = mul(mvp, daxa_f32vec4(position, 1.0));
  // mix between min and max size based on distance
  output.pointSize = lerp(POINT_MIN_SIZE, POINT_MAX_SIZE, 10.0 / distance);

  return output;
}

// Fragment Shader
[shader("fragment")] daxa_f32vec4 entry_fragment(
  CoarseVertex coarse_vertex : CoarseVertex) : SV_Target
{
  daxa_f32vec3 color = coarse_vertex.color;

  return daxa_f32vec4(color, 1.0);
}



static const daxa_f32vec3 LINE_COLOR = daxa_f32vec3(1.0, 0.7, 0.0);

struct VertexLineStageOutput
{
  CoarseVertex coarse_vertex : CoarseVertex;
  daxa_f32vec4 sv_position : SV_Position;
};

// Vertex  Shader
[shader("vertex")] VertexLineStageOutput entry_line_vertex(
    uint vertexID : SV_VertexID  // Current vertex index
)
{
  VertexLineStageOutput output;

  daxa_f32mat4x4 view = Ptr<CameraView>(UIL.task_head.camera).view;
  daxa_f32mat4x4 proj = Ptr<CameraView>(UIL.task_head.camera).proj;

  daxa_f32mat4x4 mvp = mul(proj, view);

  Ptr<GUIVertexLine> gui_vertices = Ptr<GUIVertexLine>(UIL.task_head.vertex_buffer);

  daxa_f32vec3 position = gui_vertices[vertexID].position;
  daxa_f32vec3 color = LINE_COLOR;

  output.coarse_vertex.color = color;
  output.sv_position = mul(mvp, daxa_f32vec4(position, 1.0));

  return output;
}

// Fragment Shader
[shader("fragment")] daxa_f32vec4 entry_line_fragment(
  CoarseVertex coarse_vertex : CoarseVertex) : SV_Target
{
  daxa_f32vec3 color = coarse_vertex.color;

  return daxa_f32vec4(color, 1.0);
}


static const daxa_f32vec3 AXIS_X_COLOR = daxa_f32vec3(1.0, 0.0, 0.0);
static const daxa_f32vec3 AXIS_Y_COLOR = daxa_f32vec3(0.0, 1.0, 0.0);
static const daxa_f32vec3 AXIS_Z_COLOR = daxa_f32vec3(0.0, 0.0, 1.0);

// Vertex  Shader
[shader("vertex")] VertexLineStageOutput entry_axes_vertex(
    uint vertexID : SV_VertexID  // Current vertex index
)
{
  VertexLineStageOutput output;

  daxa_f32mat4x4 view = Ptr<CameraView>(UIA.task_head.camera).view;
  daxa_f32mat4x4 proj = Ptr<CameraView>(UIA.task_head.camera).proj;

  daxa_f32mat4x4 mvp = mul(proj, view);

  Ptr<GUIVertexLine> gui_vertices = Ptr<GUIVertexLine>(UIA.task_head.vertex_buffer);

  daxa_f32vec3 position = gui_vertices[vertexID].position;
  daxa_f32vec3 color = vertexID % 6 == 0 || vertexID % 6 == 1 ? AXIS_X_COLOR : vertexID % 6 == 2 || vertexID % 6 == 3 ? AXIS_Y_COLOR : AXIS_Z_COLOR;

  output.coarse_vertex.color = color;
  output.sv_position = mul(mvp, daxa_f32vec4(position, 1.0));

  return output;
}

// Fragment Shader
[shader("fragment")] daxa_f32vec4 entry_axes_fragment(
  CoarseVertex coarse_vertex : CoarseVertex) : SV_Target
{
  daxa_f32vec3 color = coarse_vertex.color;

  return daxa_f32vec4(color, 1.0);
}