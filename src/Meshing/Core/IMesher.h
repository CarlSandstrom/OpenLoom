#pragma once

#include <string>

namespace Geometry3D
{
class GeometryCollection3D;
}
namespace Topology3D
{
class Topology3D;
}
namespace Meshing
{
class MeshingContext;
class IQualityController;
} // namespace Meshing

namespace Meshing
{

// Interface for meshing strategy implementations.
// Each mesher can optionally rely on a quality controller for validation/refinement.
class IMesher
{
public:
    explicit IMesher(const IQualityController* qualityController) :
        qualityController_(qualityController) {}
    virtual ~IMesher() = default;

    // Generate or regenerate a mesh in the provided context.
    // Implementations may clear existing data or refine in-place.
    virtual void generate(MeshingContext& context) = 0;

    // Name/identifier for logging or selection.
    virtual std::string getName() const = 0;

protected:
    // Non-owning pointer to quality controller (may be nullptr if not provided)
    const IQualityController* qualityController_ = nullptr;
};

} // namespace Meshing
