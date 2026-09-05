#pragma once
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace beat_box_diagnostics
{
inline bool parse_boolean(std::string_view name, char const *value)
{
    if (value == nullptr) return false;
    std::string normalized(value);
    for (char &c : normalized)
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    if (normalized == "1" || normalized == "true" || normalized == "on") return true;
    if (normalized == "0" || normalized == "false" || normalized == "off") return false;
    throw std::invalid_argument(std::string(name) + " must be 0/1, false/true or off/on");
}

struct Options
{
    bool full_barriers;
    bool tgs_serial;
};

// One immutable snapshot per process; graph recording and callbacks share it.
inline Options const &options()
{
    static Options const value = [] {
        Options result{
            parse_boolean("BB_SYNC_FULL_BARRIERS", std::getenv("BB_SYNC_FULL_BARRIERS")),
            parse_boolean("BB_TGS_SERIAL", std::getenv("BB_TGS_SERIAL"))};
        std::clog << "[DIAGNOSTICS] full_barriers=" << result.full_barriers
                  << " tgs_serial=" << result.tgs_serial << '\n';
        return result;
    }();
    return value;
}
}
