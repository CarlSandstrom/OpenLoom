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
    computer_(context.getMeshData()),
    context_(context),
    meshData_(context.getMeshData()),
    meshMutator_(context.getMutator())
{
}

} // namespace Meshing
