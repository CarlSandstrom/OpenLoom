#include "Meshing/Core/3D/General/SizingField3D.h"

#include "Common/Exceptions/MeshException.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Meshing
{

SizingField3D::SizingField3D(std::vector<SizingSource> sources, double gradientLimit) :
    sources_(std::move(sources)),
    gradientLimit_(gradientLimit),
    minimumSourceSize_(std::numeric_limits<double>::infinity())
{
    if (sources_.empty())
    {
        OPENLOOM_THROW_MESH(INVALID_OPERATION,
                            "SizingField3D needs at least one source; an empty field would "
                            "impose no size constraint anywhere");
    }

    if (!(gradientLimit_ > 0.0) || !std::isfinite(gradientLimit_))
    {
        OPENLOOM_THROW_MESH(INVALID_OPERATION,
                            "SizingField3D gradient limit must be finite and strictly positive");
    }

    for (const auto& source : sources_)
    {
        if (!(source.size > 0.0) || !std::isfinite(source.size))
        {
            OPENLOOM_THROW_MESH(INVALID_OPERATION,
                                "SizingField3D source sizes must be finite and strictly positive");
        }

        minimumSourceSize_ = std::min(minimumSourceSize_, source.size);
    }
}

double SizingField3D::evaluate(const Point3D& point) const
{
    double size = std::numeric_limits<double>::infinity();

    for (const auto& source : sources_)
    {
        size = std::min(size, source.size + gradientLimit_ * (point - source.position).norm());
    }

    return size;
}

} // namespace Meshing
