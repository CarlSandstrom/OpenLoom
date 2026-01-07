#pragma once

#include "../Base/IFace2D.h"
#include "OpenCascade2DEdgeLoop.h"
#include <TopoDS_Face.hxx>
#include <memory>
#include <vector>

namespace Geometry2D
{

class OpenCascade2DFace : public IFace2D
{
public:
    explicit OpenCascade2DFace(std::unique_ptr<OpenCascade2DEdgeLoop> outerEdgeLoop);

    void addHole(std::unique_ptr<OpenCascade2DEdgeLoop> holeEdgeLoop);

    const IEdgeLoop2D& getOuterEdgeLoop() const override;
    size_t getHoleCount() const override;
    const IEdgeLoop2D& getHoleEdgeLoop(size_t index) const override;
    bool hasHoles() const override;

    PointLocation classify(const Meshing::Point2D& point) const override;
    PointLocation classify(double u, double v) const override;

private:
    void buildFace() const;

    std::unique_ptr<OpenCascade2DEdgeLoop> outerEdgeLoop_;
    std::vector<std::unique_ptr<OpenCascade2DEdgeLoop>> holeEdgeLoops_;
    mutable TopoDS_Face face_;
    mutable bool faceBuilt_;
};

} // namespace Geometry2D
