#include "window.hpp"
#include "gpu_context.hpp"

int main(int argc, char const *argv[])
{
    // Create a window
    auto window = AppWindow("Beat Box", 860, 640);

    auto gpu = GPUcontext(window);

    while (!window.should_close())
    {
        window.update();
    }

    return 0;
}