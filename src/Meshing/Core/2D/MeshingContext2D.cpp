#include "MeshingContext2D.h"
#include "PeriodicMeshData2D.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "MeshOperations2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include <set>

namespace Meshing
{

MeshingContext2D::MeshingContext2D(
    std::unique_ptr<Geometry2D::GeometryCollection2D> geometry,
    std::unique_ptr<Topology2D::Topology2D> topology) :
    geometry_(std::move(geometry)),
    topology_(std::move(topology))
{
    ensureInitialized();
}

MeshingContext2D::~MeshingContext2D() = default;

MeshingContext2D::MeshingContext2D(MeshingContext2D&&) noexcept = default;
MeshingContext2D& MeshingContext2D::operator=(MeshingContext2D&&) noexcept = default;

void MeshingContext2D::setPeriodicConfig(const PeriodicDomainConfig& config)
{
    periodicConfig_ = config;
    // Reset lazy-initialized objects so they are rebuilt with the new config.
    meshOperations_.reset();
    periodicData_.reset();
}

MeshData2D& MeshingContext2D::getMeshData()
{
    ensureInitialized();
    return *meshData_;
}

const MeshData2D& MeshingContext2D::getMeshData() const
{
    return *meshData_;
}

MeshOperations2D& MeshingContext2D::getOperations()
{
    ensureInitialized();
    return *meshOperations_;
}

void MeshingContext2D::ensureInitialized()
{
    if (!meshData_)
    {
        meshData_ = std::make_unique<MeshData2D>();
    }
    if (!periodicData_ && periodicConfig_)
    {
        periodicData_ = std::make_unique<PeriodicMeshData2D>(*meshData_, *periodicConfig_);
    }
    if (!meshMutator_)
    {
        meshMutator_ = std::make_unique<MeshMutator2D>(*meshData_);
    }
    if (!meshOperations_)
    {
        meshOperations_ = std::make_unique<MeshOperations2D>(*meshData_, periodicData_.get());
    }
}

} // namespace Meshing
