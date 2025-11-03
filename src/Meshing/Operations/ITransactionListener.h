#pragma once
#include <array>
#include <memory>

namespace Meshing
{

class IElement;

/**
 * @brief Interface for objects that need to be notified of mesh changes
 *
 * This allows MeshTransaction to record changes without being a friend class
 */
class ITransactionListener
{
public:
    virtual ~ITransactionListener() = default;

    // Called by MeshData when operations occur
    virtual void onElementAdded(size_t elementId) = 0;
    virtual void onElementRemoved(size_t elementId, std::unique_ptr<IElement> clone) = 0;
    virtual void onNodeAdded(size_t nodeId) = 0;
    virtual void onNodeModified(size_t nodeId, const std::array<double, 3>& oldCoordinates) = 0;
    virtual void onNodeRemoved(size_t nodeId, const std::array<double, 3>& coordinates) = 0;
};

} // namespace Meshing