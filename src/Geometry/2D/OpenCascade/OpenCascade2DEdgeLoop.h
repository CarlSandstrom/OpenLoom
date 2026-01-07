#pragma once

#include "../Base/IEdgeLoop2D.h"
#include <TopoDS_Wire.hxx>
#include <string>
#include <vector>

namespace Geometry2D
{

class GeometryCollection2D;

class OpenCascade2DEdgeLoop : public IEdgeLoop2D
{
public:
    OpenCascade2DEdgeLoop(const std::vector<std::string>& edgeIds,
                          const GeometryCollection2D& geometry);

    std::vector<std::string> getEdgeIds() const override;
    bool isClosed() const override;
    bool isCounterClockwise() const override;

    const TopoDS_Wire& getWire() const { return wire_; }

private:
    void buildWire(const GeometryCollection2D& geometry);
    double computeSignedArea(const GeometryCollection2D& geometry) const;

    std::vector<std::string> edgeIds_;
    TopoDS_Wire wire_;
    bool closed_;
    bool counterClockwise_;
};

} // namespace Geometry2D
