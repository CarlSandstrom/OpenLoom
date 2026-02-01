#include "StepReader2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt2d.hxx>
#include <spdlog/spdlog.h>
#include <stdexcept>

using namespace Readers;

StepReader2D::StepReader2D(const std::string& filePath)
{
    geometry_ = std::make_unique<Geometry2D::GeometryCollection2D>();
    loadStep(filePath);
    extractFace();
    buildTopologyAndGeometry();
}

const Topology2D::Topology2D& StepReader2D::getTopology() const
{
    return *topology_;
}

const Geometry2D::GeometryCollection2D& StepReader2D::getGeometry() const
{
    return *geometry_;
}

std::unique_ptr<Topology2D::Topology2D> StepReader2D::takeTopology()
{
    return std::move(topology_);
}

std::unique_ptr<Geometry2D::GeometryCollection2D> StepReader2D::takeGeometry()
{
    return std::move(geometry_);
}

void StepReader2D::loadStep(const std::string& filePath)
{
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(filePath.c_str());

    if (status != IFSelect_RetDone)
    {
        throw std::runtime_error("Failed to read STEP file: " + filePath);
    }

    reader.TransferRoots();
    shape_ = reader.OneShape();

    spdlog::info("STEP file loaded: {}", filePath);
}

void StepReader2D::extractFace()
{
    TopExp_Explorer explorer(shape_, TopAbs_FACE);
    if (!explorer.More())
    {
        throw std::runtime_error("No faces found in STEP file");
    }

    face_ = TopoDS::Face(explorer.Current());
    spdlog::info("Extracted face from STEP file");
}

void StepReader2D::buildTopologyAndGeometry()
{
    // Process the outer wire
    TopoDS_Wire outerWire = BRepTools::OuterWire(face_);
    std::vector<std::string> outerEdgeLoop;
    processWire(outerWire, face_, outerEdgeLoop);

    // Process inner wires (holes)
    std::vector<std::vector<std::string>> holeEdgeLoops;
    TopExp_Explorer wireExplorer(face_, TopAbs_WIRE);
    while (wireExplorer.More())
    {
        TopoDS_Wire wire = TopoDS::Wire(wireExplorer.Current());
        if (!wire.IsSame(outerWire))
        {
            std::vector<std::string> holeEdgeLoop;
            processWire(wire, face_, holeEdgeLoop);
            holeEdgeLoops.push_back(std::move(holeEdgeLoop));
        }
        wireExplorer.Next();
    }

    // Build topology corners from accumulated connectivity
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    for (const auto& [vertexId, edgeIds] : vertexEdgeConnectivity_)
    {
        topoCorners.emplace(vertexId, Topology2D::Corner2D(vertexId, edgeIds));
    }

    topology_ = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges_, outerEdgeLoop, holeEdgeLoops);

    spdlog::info("Built 2D topology: {} corners, {} edges, {} holes",
                 topoCorners.size(), topoEdges_.size(), holeEdgeLoops.size());
}

void StepReader2D::processWire(const TopoDS_Wire& wire,
                                const TopoDS_Face& face,
                                std::vector<std::string>& edgeLoop)
{
    BRepTools_WireExplorer wireExp(wire, face);

    while (wireExp.More())
    {
        TopoDS_Edge edge = wireExp.Current();

        // Get the 2D curve on the face
        double first, last;
        Handle(Geom2d_Curve) curve2d = BRep_Tool::CurveOnSurface(edge, face, first, last);
        if (curve2d.IsNull())
        {
            spdlog::warn("Edge has no 2D curve on face, skipping");
            wireExp.Next();
            continue;
        }

        // Create trimmed curve with correct parameter bounds
        Handle(Geom2d_TrimmedCurve) trimmedCurve = new Geom2d_TrimmedCurve(curve2d, first, last);

        std::string edgeId = "edge_" + std::to_string(edgeCounter_++);
        geometry_->addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(trimmedCurve, edgeId));
        edgeLoop.push_back(edgeId);

        // Get start and end vertices
        TopoDS_Vertex startVertex, endVertex;
        TopExp::Vertices(edge, startVertex, endVertex, Standard_True);

        // Handle edge orientation relative to wire
        if (edge.Orientation() == TopAbs_REVERSED)
        {
            std::swap(startVertex, endVertex);
        }

        std::string startId = findOrCreateVertex(startVertex, face);
        std::string endId = findOrCreateVertex(endVertex, face);

        // Create topology edge
        topoEdges_.emplace(edgeId, Topology2D::Edge2D(edgeId, startId, endId));

        // Track vertex-edge connectivity
        vertexEdgeConnectivity_[startId].insert(edgeId);
        vertexEdgeConnectivity_[endId].insert(edgeId);

        wireExp.Next();
    }
}

std::string StepReader2D::findOrCreateVertex(const TopoDS_Vertex& vertex,
                                              const TopoDS_Face& face)
{
    for (const auto& [knownVertex, id] : knownVertices_)
    {
        if (vertex.IsSame(knownVertex))
        {
            return id;
        }
    }

    gp_Pnt2d uv = BRep_Tool::Parameters(vertex, face);

    std::string vertexId = "vertex_" + std::to_string(vertexCounter_++);
    geometry_->addCorner(std::make_unique<Geometry2D::OpenCascade2DCorner>(uv, vertexId));

    knownVertices_.emplace_back(vertex, vertexId);
    return vertexId;
}
