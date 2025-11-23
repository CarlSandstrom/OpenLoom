#include "BoundingBox2D.h"

namespace Common
{

BoundingBox2D::BoundingBox2D(double uMin, double uMax, double vMin, double vMax) :
    uMin_(uMin),
    uMax_(uMax),
    vMin_(vMin),
    vMax_(vMax)
{
}

double BoundingBox2D::getUMin() const
{
    return uMin_;
}

double BoundingBox2D::getUMax() const
{
    return uMax_;
}

double BoundingBox2D::getVMin() const
{
    return vMin_;
}

double BoundingBox2D::getVMax() const
{
    return vMax_;
}

} // namespace Common
