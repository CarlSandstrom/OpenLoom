#include "Node2D.h"

namespace Meshing
{

Node2D::Node2D(const Point2D& coordinates) :
    coordinates_(coordinates)
{
}

Node2D::Node2D(const Point2D& coordinates, double edgeParameter, const std::string& geometryId) :
    coordinates_(coordinates),
    edgeParameter_(edgeParameter),
    geometryId_(geometryId)
{
}

} // namespace Meshing
