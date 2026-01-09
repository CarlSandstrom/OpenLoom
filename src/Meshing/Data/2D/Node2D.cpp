#include "Node2D.h"

namespace Meshing
{

Node2D::Node2D(const Point2D& coordinates) :
    coordinates_(coordinates)
{
}

Node2D::Node2D(const Point2D& coordinates, const std::vector<double>& edgeParameters, const std::vector<std::string>& geometryIds) :
    coordinates_(coordinates),
    edgeParameters_(edgeParameters),
    geometryIds_(geometryIds)
{
}

} // namespace Meshing
