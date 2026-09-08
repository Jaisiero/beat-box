#pragma once
#include <daxa/utils/imgui.hpp>
#include "performance_metrics.hpp"

namespace beatbox {
struct PerformanceOverlay {
  ImGuiContext *context = nullptr;
  daxa::ImGuiRenderer renderer;
  PerformanceRates rates;
  PerformanceMetric frame;
  void create(daxa::Device device, daxa::Format format) {
    context = ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    renderer = daxa::ImGuiRenderer({.device = device, .format = format, .imgui_context = context, .use_custom_config = false});
  }
  void draw(daxa::CommandRecorder &recorder, daxa::ImageId image, unsigned width, unsigned height,
            PerformanceMetric const &sim, PerformanceMetric const &render, char const *solver, bool paused) {
    ImGui::SetCurrentContext(context);
    auto &io = ImGui::GetIO(); io.DisplaySize = ImVec2(float(width), float(height));
    io.DeltaTime = float(1.0 / std::max(rates.render_fps, 1.0));
    ImGui::NewFrame();
    float const scale = std::clamp(float(height) / 1080.0f, 1.0f, 2.0f);
    ImGui::SetNextWindowPos(ImVec2(14 * scale, 14 * scale));
    ImGui::SetNextWindowBgAlpha(0.85f);
    ImGui::Begin("Performance", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                 ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing);
    ImGui::SetWindowFontScale(scale);
    ImGui::Text("PERFORMANCE  %s%s", solver, paused ? "  [PAUSED]" : "");
    ImGui::Separator();
    ImGui::Text("SIM GPU     %6.2f ms   %6.1f Hz", sim.current_ms, rates.sim_hz);
    ImGui::Text("RENDER GPU  %6.2f ms   %6.1f FPS", render.current_ms, rates.render_fps);
    ImGui::Text("WORST STEP  %6.2f ms   RENDER %6.2f ms", sim.worst_ms, render.worst_ms);
    ImGui::Text("FRAME       %6.2f ms   WORST  %6.2f ms", frame.current_ms, frame.worst_ms);
    ImGui::TextDisabled("Session peaks; resize frames excluded");
    ImGui::End(); ImGui::Render();
    // Daxa's ImGui utility owns its vertex/index buffers outside this graph.
    // Order previous MAIN shader/index reads before its next transfer overwrite.
    recorder.pipeline_barrier({.src_access = daxa::AccessConsts::VERTEX_SHADER_READ | daxa::AccessConsts::INDEX_INPUT_READ,
                               .dst_access = daxa::AccessConsts::TRANSFER_WRITE});
    renderer.record_commands({.draw_data = ImGui::GetDrawData(), .recorder = recorder,
                              .target_image = image, .size_x = width, .size_y = height});
  }
  void destroy() { renderer = {}; ImGui::DestroyContext(context); context = nullptr; }
};
}
