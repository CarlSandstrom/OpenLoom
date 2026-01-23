#pragma once

#include <cstdlib>
#include <cstring>

namespace cMesh
{

/// Debug flag identifiers for runtime debugging control.
/// Enable via environment variables: CMESH_DEBUG_<FLAG_NAME>=1
enum class DebugFlag
{
    CHECK_MESH_EACH_ITERATION,
    EXPORT_MESH_EACH_ITERATION,
    VERBOSE_TRIANGULATION,
    LOG_CAVITY_OPERATIONS
};

/// Check if a debug flag is enabled via environment variable.
inline bool isDebugEnabled(DebugFlag flag)
{
    const char* envName = nullptr;

    switch (flag)
    {
    case DebugFlag::CHECK_MESH_EACH_ITERATION:
        envName = "CHECK_MESH_EACH_ITERATION";
        break;
    case DebugFlag::EXPORT_MESH_EACH_ITERATION:
        envName = "EXPORT_MESH_EACH_ITERATION";
        break;
    case DebugFlag::VERBOSE_TRIANGULATION:
        envName = "VERBOSE_TRIANGULATION";
        break;
    case DebugFlag::LOG_CAVITY_OPERATIONS:
        envName = "LOG_CAVITY_OPERATIONS";
        break;
    default:
        return false;
    }

    const char* val = std::getenv(envName);
    return val != nullptr && std::strcmp(val, "1") == 0;
}

} // namespace cMesh

// Convenience macro for concise debug checks
#define CMESH_DEBUG_ENABLED(flag) cMesh::isDebugEnabled(cMesh::DebugFlag::flag)
