#pragma once

namespace Common
{

class BoundingBox2D
{
public:
    BoundingBox2D(double uMin, double uMax, double vMin, double vMax);

    double getUMin() const;
    double getUMax() const;
    double getVMin() const;
    double getVMax() const;

private:
    double uMin_;
    double uMax_;
    double vMin_;
    double vMax_;
};

} // namespace Common
