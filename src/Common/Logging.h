#pragma once

#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

namespace Common
{

inline void initLogging()
{
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");
    spdlog::cfg::load_env_levels();
}

} // namespace Common
