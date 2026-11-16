#pragma once

#include "Geometry/2D/Base/ICorner2D.h"
#include <gp_Pnt2d.hxx>

namespace Geometry2D
{

/**
 * @brief OpenCASCADE implementation of ICorner2D, backed by a gp_Pnt2d in
 * parametric (u,v) space
 */
class OpenCascade2DCorner : public ICorner2D
{
public:
    explicit OpenCascade2DCorner(const gp_Pnt2d& point,
                                 const std::string& id = "");

    Meshing::Point2D getPoint() const override;
    std::string getId() const override;

private:
    gp_Pnt2d point_;
    std::string id_;
};

} // namespace Geometry2D
