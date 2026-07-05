#define _CRT_SECURE_NO_WARNINGS // std::getenv (headless measurement hooks below)
#include <cstdlib>
#include <cstdio>

#include "defines.hpp"
#include "window_manager.hpp"
#include "gpu_context.hpp"
#include "task_manager.hpp"
#include "renderer_manager.hpp"
#include "ray_tracing_pipeline.hpp"
#include "acceleration_structure_manager.hpp"
#include "scene_manager.hpp"
#include "input_manager.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "status_manager.hpp"
#include "gui_manager.hpp"
#include "image_manager.hpp"


#include "shared.inl"

using namespace beatbox;

int main()
{
  setvbuf(stdout, nullptr, _IONBF, 0); // unbuffered so captured [PERF]/[AUTOSTART] flush in real time
  try {
    // Input manager
    InputManager input_manager;
    // Window
    WindowManager window("Beat Box", input_manager, 860, 640);
    // GPU context
    auto gpu = std::make_shared<GPUcontext>("RT device", "Swapchain", window);
    // Task manager (pipeline manager)
    auto task_manager = std::make_shared<TaskManager>("Pipeline Manager", gpu);
    // Camera manager
    auto camera_manager = std::make_shared<CameraManager>(gpu->device);
    // Acceleration structure manager
    auto accel_struct_mngr = std::make_shared<AccelerationStructureManager>(gpu->device, task_manager);
    // Rigid body simulator pipeline
    auto rigid_body_manager = std::make_shared<RigidBodyManager>(gpu->device, task_manager, accel_struct_mngr);
    // Status manager
    auto status_manager = std::make_shared<StatusManager>(gpu, accel_struct_mngr, rigid_body_manager);
    // Scene manager
    auto scene_manager = std::make_shared<SceneManager>("Scene Manager", gpu->device, accel_struct_mngr, rigid_body_manager, status_manager, task_manager);
    // GUI manager
    auto gui_manager = std::make_shared<GUIManager>(gpu, window, task_manager, rigid_body_manager);
    // Image manager
    auto image_manager = std::make_shared<ImageManager>(gpu, task_manager);

    // Primary tracing pipeline
    auto RT_pipeline = std::make_shared<RayTracingPipeline>(task_manager->create_ray_tracing(MainRayTracingPipeline{}.info), gpu->device);
    // Renderer
    auto renderer = std::make_shared<RendererManager>(gpu, task_manager, window, camera_manager, accel_struct_mngr, rigid_body_manager, scene_manager, status_manager, gui_manager, image_manager);

    // Every create() reports failure via its bool return; discarding them let the app continue
    // into the render loop on half-initialized state (GPU faults later, far from the cause).
    // Abort with the failing manager's name instead (review v3).
    auto check_init = [](bool ok, char const *name) {
      if (!ok) {
        std::cerr << "CRITICAL ERROR: " << name << "::create() failed! Exiting application." << std::endl;
        std::exit(-1);
      }
    };
    // Create image manager
    check_init(image_manager->create(), "ImageManager");
    // Create camera manager
    check_init(camera_manager->create("Camera Manager"), "CameraManager");
    // Create input manager which depends on camera manager and window
    check_init(input_manager.create(camera_manager, status_manager), "InputManager");
    // Create GUI manager
    check_init(gui_manager->create(renderer, status_manager), "GUIManager");
    // Create task graph
    check_init(renderer->create("Ray Tracing Task Graph", RT_pipeline, RT_pipeline->build_SBT()), "RendererManager");
    // Create rigid body simulator
    check_init(rigid_body_manager->create("Rigid Body Manager", renderer, gui_manager), "RigidBodyManager");
    // Create acceleration structure manager
    check_init(accel_struct_mngr->create(renderer, rigid_body_manager, gui_manager), "AccelerationStructureManager");
    // Create status manager
    check_init(status_manager->create(), "StatusManager");
    // Create scene manager
    check_init(scene_manager->create(), "SceneManager");

    // Load scene
    if(!scene_manager->load_scene()) {
      std::cerr << "CRITICAL ERROR: Failed to load scene! Exiting application." << std::endl;
      return -1;
    }

    std::cout << "SUCCESS: Application initialized successfully! Entering main graphics and simulation loop." << std::endl;

    // Headless measurement hooks: BB_SOLVER=0/1/2 -> PGS/PGS_SOFT/AVBD, BB_AUTOSTART set -> start the sim
    // (lets the app run + emit [PERF] without keyboard input, for self-measurement of sim= ms).
    if (const char* solver_env = std::getenv("BB_SOLVER")) {
      status_manager->set_solver(static_cast<SimSolverType>(std::atoi(solver_env)));
      std::cout << "[AUTOSTART] solver=" << std::atoi(solver_env) << std::endl;
    }
    if (std::getenv("BB_AUTOSTART")) {
      status_manager->switch_simulating();
      std::cout << "[AUTOSTART] simulating ON" << std::endl;
    }
    if (std::getenv("BB_NO_SLEEP")) {
      rigid_body_manager->clear_sim_flags(SimFlag::SLEEPING_ENABLED);
      std::cout << "[AUTOSTART] SLEEPING DISABLED (repro: a stuck/buzzing pair stays observable instead of sleeping away)" << std::endl;
    }
    if (std::getenv("BB_DETERMINISTIC")) {
      rigid_body_manager->set_sim_flags(SimFlag::DETERMINISTIC);
      std::cout << "[AUTOSTART] DETERMINISTIC mode ON (post-stab skipped -> cross-launch reproducible)" << std::endl;
    }
    // Enable the determinism debug hashes (dbg_*) only when actually measuring determinism; otherwise
    // they stay off so their per-body/per-manifold atomics + chain-walk don't cost in normal runs.
    if (std::getenv("BB_DET_STEPS") || std::getenv("BB_DET_INPROC") || std::getenv("BB_DETERMINISTIC")) {
      rigid_body_manager->set_sim_flags(SimFlag::DET_HASHES);
      std::cout << "[AUTOSTART] determinism debug hashes ON" << std::endl;
    }

    // D1-offline: BB_COMPILE_ONLY warms the SPIR-V cache and exits. All pipelines were already compiled
    // by the create() calls above (the ~100s cold Slang->SPIR-V pass), so spirv_cache/ is now populated
    // next to the exe — skip the render loop so a build step (the warm_shader_cache CMake target) can
    // pre-warm the cache and make the FIRST real launch fast (~2s) instead of paying ~100s then.
    int render_rc = 0;
    if (std::getenv("BB_COMPILE_ONLY")) {
      std::cout << "[COMPILE] shader cache warmed; exiting without rendering (BB_COMPILE_ONLY)." << std::endl;
    } else {
      // Main loop (returns non-zero if a BB_ASSERT_* metric threshold failed — for headless A/B/CI)
      render_rc = renderer->render();
    }

    // Cleanup
    scene_manager->destroy();
    status_manager->destroy();
    rigid_body_manager->destroy();
    gui_manager->destroy();
    accel_struct_mngr->destroy();
    input_manager.destroy();
    camera_manager->destroy();
    image_manager->destroy();

    return render_rc;
  }
  catch (const std::exception& e) {
    std::string const msg = e.what();
    std::cerr << "\nCRITICAL ERROR (Standard Exception Thrown): " << msg << std::endl;
    // F2: name the likely GPU out-of-memory case explicitly (the AS pools + per-frame buffers are the
    // large device allocations); a raw daxa/Vulkan message is otherwise opaque.
    if (msg.find("memory") != std::string::npos || msg.find("MEMORY") != std::string::npos ||
        msg.find("ALLOCATION") != std::string::npos || msg.find("allocat") != std::string::npos) {
      std::cerr << "This looks like a GPU out-of-memory failure. The acceleration-structure pools and "
                   "per-frame buffers are the largest device allocations (see BLAS_POOL_BUDGET / "
                   "MAX_RIGID_BODY_COUNT); reduce them or free VRAM." << std::endl;
    }
    std::cerr << "Please verify device/Vulkan features and initialization parameters." << std::endl;
    return 1;
  }
  catch (...) {
    std::cerr << "\nCRITICAL ERROR: An unknown unhandled exception occurred during application execution!" << std::endl;
    return 1;
  }
}
