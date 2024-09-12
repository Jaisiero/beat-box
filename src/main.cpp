#include "window.hpp"

int main(int argc, char const *argv[])
{
    // Create a window
    auto window = AppWindow("Beat Box", 860, 640);

    // Daxa code goes here...

    while (!window.should_close())
    {
        window.update();
    }

    return 0;
}