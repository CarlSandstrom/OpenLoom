#pragma once

namespace Common
{

class BoundingBox3D
{
public:
    BoundingBox3D(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax);

    double getXMin() const;
    double getXMax() const;
    double getYMin() const;
    double getYMax() const;
    double getZMin() const;
    double getZMax() const;

private:
    double xMin_;
    double xMax_;
    double yMin_;
    double yMax_;
    double zMin_;
    double zMax_;
};

} // namespace Common
