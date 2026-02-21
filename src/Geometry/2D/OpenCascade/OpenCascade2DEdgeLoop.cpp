#include "OpenCascade2DEdgeLoop.h"

#include "../Base/GeometryCollection2D.h"
#include "OpenCascade2DEdge.h"

#include "Common/Exceptions/GeometryException.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeEdge2d.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRep_Builder.hxx>
#include <Geom2d_Line.hxx>
#include <Geom_Plane.hxx>
#include <cmath>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

namespace Geometry2D
{

OpenCascade2DEdgeLoop::OpenCascade2DEdgeLoop(const std::vector<std::string>& edgeIds,
                                             const GeometryCollection2D& geometry) :
    edgeIds_(edgeIds),
    closed_(false),
    counterClockwise_(true)
{
    if (edgeIds_.empty())
    {
        OPENLOOM_THROW_CODE(OpenLoom::GeometryException,
                         OpenLoom::GeometryException::ErrorCode::EMPTY_COLLECTION,
                         "Edge list cannot be empty");
    }

    buildWire(geometry);

    counterClockwise_ = computeSignedArea(geometry) > 0.0;

    // Check if closed by comparing first edge start with last edge end
    const IEdge2D* firstEdge = geometry.getEdge(edgeIds_.front());
    const IEdge2D* lastEdge = geometry.getEdge(edgeIds_.back());

    if (firstEdge && lastEdge)
    {
        Meshing::Point2D start = firstEdge->getStartPoint();
        Meshing::Point2D end = lastEdge->getEndPoint();
        double dist = std::sqrt((start.x() - end.x()) * (start.x() - end.x()) +
                                (start.y() - end.y()) * (start.y() - end.y()));
        closed_ = (dist < 1e-10);
    }
}

std::vector<std::string> OpenCascade2DEdgeLoop::getEdgeIds() const
{
    return edgeIds_;
}

bool OpenCascade2DEdgeLoop::isClosed() const
{
    return closed_;
}

bool OpenCascade2DEdgeLoop::isCounterClockwise() const
{
    return counterClockwise_;
}

void OpenCascade2DEdgeLoop::buildWire(const GeometryCollection2D& geometry)
{
    // Define the XY plane at Z=0 for 2D operations
    Handle(Geom_Plane) surface = new Geom_Plane(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)));

    BRepBuilderAPI_MakeWire wireBuilder;

    for (const std::string& edgeId : edgeIds_)
    {
        const IEdge2D* edge = geometry.getEdge(edgeId);
        if (!edge)
        {
            OPENLOOM_THROW_ENTITY_NOT_FOUND("Edge", edgeId);
        }

        // Try to get OpenCASCADE curve if available
        const OpenCascade2DEdge* occEdge = dynamic_cast<const OpenCascade2DEdge*>(edge);

        if (occEdge)
        {
            // Use the actual 2D curve and create a 3D edge on the surface
            BRepBuilderAPI_MakeEdge edgeBuilder(
                occEdge->getCurve(),
                surface,
                occEdge->getFirstParameter(),
                occEdge->getLastParameter());

            if (!edgeBuilder.IsDone())
            {
                OPENLOOM_THROW_CODE(OpenLoom::GeometryException,
                                 OpenLoom::GeometryException::ErrorCode::WIRE_BUILDING_FAILED,
                                 "Failed to create edge: " + edgeId);
            }

            wireBuilder.Add(edgeBuilder.Edge());
        }
        else
        {
            // Create a line directly from 3D points on Z=0 plane
            Meshing::Point2D startPt = edge->getStartPoint();
            Meshing::Point2D endPt = edge->getEndPoint();

            gp_Pnt p1(startPt.x(), startPt.y(), 0.0);
            gp_Pnt p2(endPt.x(), endPt.y(), 0.0);

            BRepBuilderAPI_MakeEdge edgeBuilder(p1, p2);

            if (!edgeBuilder.IsDone())
            {
                OPENLOOM_THROW_CODE(OpenLoom::GeometryException,
                                 OpenLoom::GeometryException::ErrorCode::WIRE_BUILDING_FAILED,
                                 "Failed to create linear edge: " + edgeId);
            }

            wireBuilder.Add(edgeBuilder.Edge());
        }
    }

    if (!wireBuilder.IsDone())
    {
        OPENLOOM_THROW_CODE(OpenLoom::GeometryException,
                         OpenLoom::GeometryException::ErrorCode::WIRE_BUILDING_FAILED,
                         "Failed to build wire");
    }

    wire_ = wireBuilder.Wire();
}

double OpenCascade2DEdgeLoop::computeSignedArea(const GeometryCollection2D& geometry) const
{
    // Use shoelace formula on edge endpoints
    double signedArea = 0.0;

    for (const std::string& edgeId : edgeIds_)
    {
        const IEdge2D* edge = geometry.getEdge(edgeId);
        if (!edge)
        {
            continue;
        }

        Meshing::Point2D p1 = edge->getStartPoint();
        Meshing::Point2D p2 = edge->getEndPoint();

        signedArea += (p1.x() * p2.y() - p2.x() * p1.y());
    }

    return signedArea / 2.0;
}

} // namespace Geometry2D
