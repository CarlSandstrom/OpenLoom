#pragma once

namespace Meshing
{

struct RCDTQualitySettings
{
    double maximumCircumradiusToShortestEdgeRatio = 1.0;
    double maximumChordDeviation = 0.1;
};

} // namespace Meshing
