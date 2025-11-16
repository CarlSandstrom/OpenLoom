#pragma once

#include "../Base/Corner.h"
#include <TopoDS_Vertex.hxx>

namespace Geometry
{

/**
 * @brief OpenCASCADE implementation of Corner
 */
class OpenCascadeCorner : public Corner
{
public:
    explicit OpenCascadeCorner(const TopoDS_Vertex& vertex);

    Meshing::Point3D getPoint() const override;
    std::string getId() const override;

private:
    TopoDS_Vertex vertex_;
};

} // namespace Geometry