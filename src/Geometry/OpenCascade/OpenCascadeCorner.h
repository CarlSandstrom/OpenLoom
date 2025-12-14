#pragma once

#include "../Base/Corner3D.h"
#include <TopoDS_Vertex.hxx>

namespace Geometry3D
{

/**
 * @brief OpenCASCADE implementation of Corner
 */
class OpenCascadeCorner : public Corner3D
{
public:
    explicit OpenCascadeCorner(const TopoDS_Vertex& vertex);

    Meshing::Point3D getPoint() const override;
    std::string getId() const override;

private:
    TopoDS_Vertex vertex_;
};

} // namespace Geometry3D