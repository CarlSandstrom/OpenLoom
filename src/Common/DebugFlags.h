#pragma once

#include <cstdlib>
#include <cstring>

namespace OpenLoom
{

/// Debug flag identifiers for runtime debugging control.
/// Each flag is enabled by an environment variable of the same name set to 1,
/// e.g. CHECK_MESH_EACH_ITERATION=1.
enum class DebugFlag
{
    CHECK_MESH_EACH_ITERATION,
    EXPORT_MESH_EACH_ITERATION,

    VERBOSE_TRIANGULATION,
    LOG_CAVITY_OPERATIONS
};

/// True when the flag's environment variable is set to exactly "1".
inline bool isDebugEnabled(DebugFlag flag)
{
    const char* environmentVariableName = nullptr;

    switch (flag)
    {
    case DebugFlag::CHECK_MESH_EACH_ITERATION:
        environmentVariableName = "CHECK_MESH_EACH_ITERATION";
        break;
    case DebugFlag::EXPORT_MESH_EACH_ITERATION:
        environmentVariableName = "EXPORT_MESH_EACH_ITERATION";
        break;
    case DebugFlag::VERBOSE_TRIANGULATION:
        environmentVariableName = "VERBOSE_TRIANGULATION";
        break;
    case DebugFlag::LOG_CAVITY_OPERATIONS:
        environmentVariableName = "LOG_CAVITY_OPERATIONS";
        break;
    default:
        return false;
    }

    const char* value = std::getenv(environmentVariableName);
    return value != nullptr && std::strcmp(value, "1") == 0;
}

} // namespace OpenLoom

#define OPENLOOM_DEBUG_ENABLED(flag) OpenLoom::isDebugEnabled(OpenLoom::DebugFlag::flag)
