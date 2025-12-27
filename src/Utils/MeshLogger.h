#pragma once

#include "Meshing/Data/2D/MeshData2D.h"

namespace Meshing
{

/**
 * @brief Utility class for logging mesh data
 */
class MeshLogger
{
public:
    /**
     * @brief Logs nodes and elements of a MeshData2D object
     * @param meshData The mesh data to log
     */
    static void logMeshData2D(const MeshData2D& meshData);
};

} // namespace Meshing
