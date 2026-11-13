#pragma once

#include "Geometry/3D/Base/ICorner3D.h"
#include <TopoDS_Vertex.hxx>

namespace Geometry3D
{

/**
 * @brief OpenCASCADE implementation of ICorner3D, backed by a TopoDS_Vertex
 */
class OpenCascadeCorner : public ICorner3D
{
public:
    explicit OpenCascadeCorner(const TopoDS_Vertex& vertex);

    Meshing::Point3D getPoint() const override;
    std::string getId() const override;

private:
    TopoDS_Vertex vertex_;
};

} // namespace Geometry3D