#pragma once

/**
 * @brief Convenience header that includes all mesh data components
 *
 * This header provides access to the separated mesh data components:
 * - MeshGeometry: Pure data container (nodes and elements)
 * - MeshOperations: Operations on mesh data (add/remove/modify)
 * - MeshConnectivity: Connectivity queries and validation
 *
 * Use these classes directly for better separation of concerns:
 *
 * @code
 * MeshGeometry geometry;                    // Pure data
 * MeshOperations operations(geometry);      // Operations
 * MeshConnectivity connectivity(geometry);  // Queries
 * @endcode
 */

#include "MeshConnectivity.h"
#include "MeshGeometry.h"
#include "MeshOperations.h"