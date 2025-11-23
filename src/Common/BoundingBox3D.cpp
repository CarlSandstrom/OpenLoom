#include "BoundingBox3D.h"

namespace Common
{

BoundingBox3D::BoundingBox3D(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax) :
    xMin_(xMin),
    xMax_(xMax),
    yMin_(yMin),
    yMax_(yMax),
    zMin_(zMin),
    zMax_(zMax)
{
}

double BoundingBox3D::getXMin() const
{
    return xMin_;
}

double BoundingBox3D::getXMax() const
{
    return xMax_;
}

double BoundingBox3D::getYMin() const
{
    return yMin_;
}

double BoundingBox3D::getYMax() const
{
    return yMax_;
}

double BoundingBox3D::getZMin() const
{
    return zMin_;
}

double BoundingBox3D::getZMax() const
{
    return zMax_;
}

} // namespace Common
