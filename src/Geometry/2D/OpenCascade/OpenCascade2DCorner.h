#pragma once

#include "../Base/ICorner2D.h"
#include <gp_Pnt2d.hxx>

namespace Geometry2D
{

/**
 * @brief OpenCASCADE implementation of 2D Corner in parametric space
 *
 * Represents a point in 2D parametric (u,v) space using OpenCASCADE's gp_Pnt2d.
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
