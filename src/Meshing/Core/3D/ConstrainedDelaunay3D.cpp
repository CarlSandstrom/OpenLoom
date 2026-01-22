#include "ConstrainedDelaunay3D.h"

#include "ComputerGeneral.h"
#include "Export/VtkExporter.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace Meshing
{

ConstrainedDelaunay3D::ConstrainedDelaunay3D(MeshingContext3D& context) :
    context_(context),
    meshData_(context.getMeshData()),
    meshMutator_(context.getMutator())
{
}

const TetrahedralElement* ConstrainedDelaunay3D::getTetrahedralElement(size_t id) const
{
    const auto* element = meshData_.getElement(id);
    return dynamic_cast<const TetrahedralElement*>(element);
}

} // namespace Meshing
