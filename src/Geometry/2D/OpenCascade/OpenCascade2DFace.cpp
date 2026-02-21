#include "OpenCascade2DFace.h"

#include "Common/Exceptions/GeometryException.h"
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Surface.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Wire.hxx>
#include <TopAbs_State.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

namespace Geometry2D
{

OpenCascade2DFace::OpenCascade2DFace(std::unique_ptr<OpenCascade2DEdgeLoop> outerEdgeLoop) :
    outerEdgeLoop_(std::move(outerEdgeLoop)),
    faceBuilt_(false)
{
    OPENLOOM_REQUIRE_NOT_NULL(outerEdgeLoop_, "outer edge loop");
}

void OpenCascade2DFace::addHole(std::unique_ptr<OpenCascade2DEdgeLoop> holeEdgeLoop)
{
    OPENLOOM_REQUIRE_NOT_NULL(holeEdgeLoop, "hole edge loop");

    holeEdgeLoops_.push_back(std::move(holeEdgeLoop));
    faceBuilt_ = false;
}

const IEdgeLoop2D& OpenCascade2DFace::getOuterEdgeLoop() const
{
    return *outerEdgeLoop_;
}

size_t OpenCascade2DFace::getHoleCount() const
{
    return holeEdgeLoops_.size();
}

const IEdgeLoop2D& OpenCascade2DFace::getHoleEdgeLoop(size_t index) const
{
    if (index >= holeEdgeLoops_.size())
    {
        OPENLOOM_THROW_CODE(OpenLoom::GeometryException,
                         OpenLoom::GeometryException::ErrorCode::PARAMETER_OUT_OF_RANGE,
                         "Hole index " + std::to_string(index) + " out of range (size: " + std::to_string(holeEdgeLoops_.size()) + ")");
    }
    return *holeEdgeLoops_[index];
}

bool OpenCascade2DFace::hasHoles() const
{
    return !holeEdgeLoops_.empty();
}

PointLocation OpenCascade2DFace::classify(const Meshing::Point2D& point) const
{
    return classify(point.x(), point.y());
}

PointLocation OpenCascade2DFace::classify(double u, double v) const
{
    buildFace();

    // Get the underlying surface to find proper UV parameters
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face_);
    gp_Pnt point3d(u, v, 0.0);

    // Project the 3D point onto the surface to get UV coordinates.
    // This is necessary because the face's UV space may have a different
    // origin/scale than the world XY coordinates.
    GeomAPI_ProjectPointOnSurf projector(point3d, surface);
    if (!projector.IsDone() || projector.NbPoints() == 0)
    {
        // Point is far from surface, consider it outside
        return PointLocation::Outside;
    }

    double uParam, vParam;
    projector.LowerDistanceParameters(uParam, vParam);
    gp_Pnt2d uvPoint(uParam, vParam);

    // Use BRepTopAdaptor_FClass2d for 2D classification with proper UV coordinates.
    // This classifier properly handles faces with holes (inner wires).
    BRepTopAdaptor_FClass2d classifier(face_, 1e-6);
    TopAbs_State state = classifier.Perform(uvPoint);

    switch (state)
    {
    case TopAbs_IN:
        return PointLocation::Inside;
    case TopAbs_OUT:
        return PointLocation::Outside;
    case TopAbs_ON:
        return PointLocation::OnBoundary;
    default:
        return PointLocation::Outside;
    }
}

void OpenCascade2DFace::buildFace() const
{
    if (faceBuilt_)
    {
        return;
    }

    // Create face from outer wire (on XY plane)
    const TopoDS_Wire& outerWire = outerEdgeLoop_->getWire();

    // Create face directly from the wire (OpenCASCADE infers the plane)
    BRepBuilderAPI_MakeFace faceBuilder(outerWire, Standard_True);

    if (!faceBuilder.IsDone())
    {
        OPENLOOM_THROW_CODE(OpenLoom::GeometryException,
                         OpenLoom::GeometryException::ErrorCode::WIRE_BUILDING_FAILED,
                         "Failed to create face from outer wire");
    }

    // Add hole wires.
    // For BRepBuilderAPI_MakeFace::Add(), hole wires should have OPPOSITE orientation
    // to the outer wire. If outer is CCW, holes should be CW.
    for (const auto& holeLoop : holeEdgeLoops_)
    {
        // Hole wires should have opposite orientation (clockwise for CCW outer boundary).
        // If the hole is already CCW, reverse it to make it CW.
        TopoDS_Wire holeWire = holeLoop->getWire();
        if (holeLoop->isCounterClockwise())
        {
            holeWire = TopoDS::Wire(holeWire.Reversed());
        }
        faceBuilder.Add(holeWire);

        if (!faceBuilder.IsDone())
        {
            OPENLOOM_THROW_CODE(OpenLoom::GeometryException,
                             OpenLoom::GeometryException::ErrorCode::WIRE_BUILDING_FAILED,
                             "Failed to add hole wire to face");
        }
    }

    face_ = faceBuilder.Face();

    // Use ShapeFix_Face to add missing pcurves for hole wires.
    // This is necessary because BRepBuilderAPI_MakeFace::Add() doesn't
    // automatically create pcurves for added inner wires, and
    // BRepTopAdaptor_FClass2d requires pcurves for correct classification.
    ShapeFix_Face faceFixer(face_);
    faceFixer.FixAddNaturalBoundMode() = Standard_False;
    faceFixer.FixWireTool()->FixAddPCurveMode() = Standard_True;
    faceFixer.Perform();
    face_ = faceFixer.Face();

    faceBuilt_ = true;
}

} // namespace Geometry2D
