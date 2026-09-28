#pragma once

#include "../Base/IVolume3D.h"
#include <TopoDS_Solid.hxx>

namespace Geometry3D
{

/**
 * @brief OpenCASCADE implementation of Volume (a solid body / material phase)
 */
class OpenCascadeVolume : public IVolume3D
{
public:
    explicit OpenCascadeVolume(const TopoDS_Solid& solid);

    std::string getId() const override;

private:
    TopoDS_Solid solid_;
};

} // namespace Geometry3D
