#pragma once

#include <string>
#include <utility>

#include "Common/Types.h"
#include "Edge2D.h"

namespace Geometry2D
{

/**
 * @brief A linear edge in 2D parametric space
 *
 * Simple implementation that linearly interpolates between start and end points.
 */
class LinearEdge2D : public IEdge2D
{
public:
    LinearEdge2D(const std::string& id,
                 const Meshing::Point2D& startPoint,
                 const Meshing::Point2D& endPoint);

    Meshing::Point2D getPoint(double t) const override;
    Meshing::Point2D getStartPoint() const override { return startPoint_; }
    Meshing::Point2D getEndPoint() const override { return endPoint_; }
    std::pair<double, double> getParameterBounds() const override { return {0.0, 1.0}; }
    double getLength() const override;

    std::string getId() const override { return id_; }

private:
    std::string id_;
    Meshing::Point2D startPoint_;
    Meshing::Point2D endPoint_;
};

} // namespace Geometry2D
