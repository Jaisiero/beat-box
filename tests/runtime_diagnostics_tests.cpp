#include "runtime_diagnostics.hpp"
int main()
{
    using beat_box_diagnostics::parse_boolean;
    if (parse_boolean("test", nullptr)) return 1;
    for (auto value : {"0", "false", "FALSE", "off", "Off"})
        if (parse_boolean("test", value)) return 2;
    for (auto value : {"1", "true", "TRUE", "on", "On"})
        if (!parse_boolean("test", value)) return 3;
    for (auto value : {"", "2", "tru", " false "})
    {
        try { (void)parse_boolean("test", value); return 4; }
        catch (std::invalid_argument const &) {}
    }
}
