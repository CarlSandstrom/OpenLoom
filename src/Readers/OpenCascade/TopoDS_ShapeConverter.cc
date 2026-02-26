#include "TopoDS_ShapeConverter.h"
#include "../../Geometry/3D/Base/ICorner3D.h"
#include "../../Geometry/3D/Base/IEdge3D.h"
#include "../../Geometry/3D/Base/ISurface3D.h"
#include "../../Geometry/3D/OpenCascade/OpenCascadeCorner.h"
#include "../../Geometry/3D/OpenCascade/OpenCascadeEdge.h"
#include "../../Geometry/3D/OpenCascade/OpenCascadeSurface.h"
#include <BRepTools_WireExplorer.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <optional>
#include <set>
using namespace Readers;

TopoDS_ShapeConverter::TopoDS_ShapeConverter(const TopoDS_Shape& shape) :
    shape_(shape),
    openCascadeGeometryCollection_(std::make_unique<OpenCascadeGeometryCollection>(shape))
{
    buildTopology();
    buildGeometryCollection();
}

const Topology3D::Topology3D& TopoDS_ShapeConverter::getTopology() const
{
    return *topology_;
}

const Geometry3D::GeometryCollection3D& TopoDS_ShapeConverter::getGeometryCollection() const
{
    return *geometryCollection_;
}

void TopoDS_ShapeConverter::buildTopology()
{
    createSurfaces();
    createEdges();
    createCorners();

    topology_ = std::make_unique<Topology3D::Topology3D>(std::unordered_map<std::string, Topology3D::Surface3D>(surfaces_),
                                                         std::unordered_map<std::string, Topology3D::Edge3D>(edges_),
                                                         std::unordered_map<std::string, Topology3D::Corner3D>(corners_),
                                                         std::move(seams_));
}

void TopoDS_ShapeConverter::buildGeometryCollection()
{
    auto vertexMap = openCascadeGeometryCollection_->getVertexMap();
    auto edgeMap = openCascadeGeometryCollection_->getEdgeMap();
    auto faceMap = openCascadeGeometryCollection_->getFaceMap();

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;

    for (const auto& [id, vertex] : vertexMap)
    {
        std::unique_ptr<Geometry3D::OpenCascadeCorner> corner = std::make_unique<Geometry3D::OpenCascadeCorner>(vertex);
        corners.emplace(id, std::move(corner));
    }

    for (const auto& [id, edge] : edgeMap)
    {
        std::unique_ptr<Geometry3D::OpenCascadeEdge> edgePtr = std::make_unique<Geometry3D::OpenCascadeEdge>(edge);
        edges.emplace(id, std::move(edgePtr));
    }

    for (const auto& [id, face] : faceMap)
    {
        std::unique_ptr<Geometry3D::OpenCascadeSurface> surface = std::make_unique<Geometry3D::OpenCascadeSurface>(face);
        surfaces.emplace(id, std::move(surface));
    }

    geometryCollection_ = std::make_unique<Geometry3D::GeometryCollection3D>(std::move(surfaces),
                                                                             std::move(edges),
                                                                             std::move(corners));
}

void TopoDS_ShapeConverter::createSurfaces()
{
    const auto& faceMap = openCascadeGeometryCollection_->getFaceMap();

    // Map edges to their adjacent faces. This is used for finding adjacent surfaces.
    TopTools_IndexedDataMapOfShapeListOfShape edgeToFacesMap;
    TopExp::MapShapesAndAncestors(shape_, TopAbs_EDGE, TopAbs_FACE, edgeToFacesMap);

    for (auto& [id, face] : faceMap)
    {
        std::vector<std::string> boundaryEdgeIds;
        std::vector<std::string> cornerIds;
        std::vector<std::string> adjacentSurfaceIds;

        // Find all edges of the face, preserving seam edge occurrences.
        // BRepTools_WireExplorer visits seam edges twice (once in each direction),
        // which is required to correctly close the UV domain of cylindrical/toroidal faces.
        std::set<std::string> seenEdgeIds;
        for (TopExp_Explorer wireExp(face, TopAbs_WIRE); wireExp.More(); wireExp.Next())
        {
            const TopoDS_Wire& wire = TopoDS::Wire(wireExp.Current());
            BRepTools_WireExplorer edgeExp(wire, face);
            for (; edgeExp.More(); edgeExp.Next())
            {
                const TopoDS_Edge& edge = edgeExp.Current();
                auto edgeIdOpt = openCascadeGeometryCollection_->findEdgeId(edge);
                if (!edgeIdOpt.has_value())
                    continue;
                const std::string& edgeId = edgeIdOpt.value();
                if (seenEdgeIds.count(edgeId) == 0)
                {
                    boundaryEdgeIds.push_back(edgeId);
                    seenEdgeIds.insert(edgeId);
                }
                else
                {
                    // Second occurrence: seam edge traversed in reverse — add as twin
                    boundaryEdgeIds.push_back(edgeId + "_seam");
                }
            }
        }

        // Find all vertices of the face
        TopTools_IndexedMapOfShape vertexMap;
        TopExp::MapShapes(face, TopAbs_VERTEX, vertexMap);

        for (size_t i = 1; i <= vertexMap.Extent(); ++i)
        {
            const TopoDS_Vertex& vertex = TopoDS::Vertex(vertexMap(i));
            auto vertexId = openCascadeGeometryCollection_->findVertexId(vertex);
            if (vertexId.has_value())
            {
                cornerIds.push_back(vertexId.value());
            }
        }

        // Find all adjacent surfaces of the face
        TopTools_IndexedMapOfShape edgesOfFace;
        TopExp::MapShapes(face, TopAbs_EDGE, edgesOfFace);
        for (int i = 1; i <= edgesOfFace.Extent(); ++i)
        {
            const TopoDS_Edge& edge = TopoDS::Edge(edgesOfFace(i));

            if (edgeToFacesMap.Contains(edge))
            {
                const TopTools_ListOfShape& facesOnEdge = edgeToFacesMap.FindFromKey(edge);

                // Add all faces except the current one
                for (TopTools_ListIteratorOfListOfShape it(facesOnEdge); it.More(); it.Next())
                {
                    const TopoDS_Face& adjacentFace = TopoDS::Face(it.Value());

                    // Skip the current face itself
                    if (!adjacentFace.IsSame(face))
                    {
                        auto adjacentFaceId = openCascadeGeometryCollection_->findSurfaceId(adjacentFace);
                        if (adjacentFaceId.has_value())
                        {
                            adjacentSurfaceIds.push_back(adjacentFaceId.value());
                        }
                    }
                }
            }
        }

        surfaces_.emplace(id, Topology3D::Surface3D(id, boundaryEdgeIds, cornerIds, adjacentSurfaceIds, {}));
    }
}

void TopoDS_ShapeConverter::createEdges()
{
    for (auto& [id, edge] : openCascadeGeometryCollection_->getEdgeMap())
    {
        // Get start and end vertices
        TopoDS_Vertex startVertex, endVertex;
        TopExp::Vertices(edge, startVertex, endVertex);

        auto startCornerIdOpt = openCascadeGeometryCollection_->findVertexId(startVertex);
        auto endCornerIdOpt = openCascadeGeometryCollection_->findVertexId(endVertex);

        std::string startCornerId = startCornerIdOpt.has_value() ? startCornerIdOpt.value() : "";
        std::string endCornerId = endCornerIdOpt.has_value() ? endCornerIdOpt.value() : "";

        // Find adjacent surfaces
        std::vector<std::string> adjacentSurfaceIds;
        TopTools_IndexedDataMapOfShapeListOfShape edgeToFacesMap;
        TopExp::MapShapesAndAncestors(shape_, TopAbs_EDGE, TopAbs_FACE, edgeToFacesMap);

        if (edgeToFacesMap.Contains(edge))
        {
            const TopTools_ListOfShape& facesOnEdge = edgeToFacesMap.FindFromKey(edge);
            for (TopTools_ListIteratorOfListOfShape it(facesOnEdge); it.More(); it.Next())
            {
                const TopoDS_Face& face = TopoDS::Face(it.Value());
                auto faceIdOpt = openCascadeGeometryCollection_->findSurfaceId(face);
                if (faceIdOpt.has_value())
                {
                    adjacentSurfaceIds.push_back(faceIdOpt.value());
                }
            }
        }

        edges_.emplace(id, Topology3D::Edge3D(id, startCornerId, endCornerId, adjacentSurfaceIds));
    }

    // Create synthetic seam twin edges for any "_seam" boundary entries in surfaces_.
    // A seam twin is the same physical edge traversed in reverse to close the UV domain.
    // Its corner IDs carry a "_seam" suffix to distinguish their UV positions (U + period).
    // The ID suffix is used here only as a bootstrapping signal from createSurfaces();
    // all downstream code queries SeamCollection instead.
    for (const auto& [surfId, surf] : surfaces_)
    {
        for (const auto& edgeId : surf.getBoundaryEdgeIds())
        {
            if (!edgeId.ends_with("_seam"))
                continue;
            if (edges_.count(edgeId) > 0)
                continue;

            std::string originalId = edgeId.substr(0, edgeId.size() - 5);
            auto origIt = edges_.find(originalId);
            if (origIt == edges_.end())
                continue;

            const auto& orig = origIt->second;
            edges_.emplace(edgeId,
                           Topology3D::Edge3D(edgeId,
                                              orig.getEndCornerId() + "_seam",
                                              orig.getStartCornerId() + "_seam",
                                              orig.getAdjacentSurfaceIds()));
            seams_.addPair(originalId, edgeId);
        }
    }
}

void TopoDS_ShapeConverter::createCorners()
{
    for (auto& [id, vertex] : openCascadeGeometryCollection_->getVertexMap())
    {
        // Find connected edges
        std::set<std::string> connectedEdgeIds;
        TopTools_IndexedDataMapOfShapeListOfShape vertexToEdgesMap;
        TopExp::MapShapesAndAncestors(shape_, TopAbs_VERTEX, TopAbs_EDGE, vertexToEdgesMap);

        if (vertexToEdgesMap.Contains(vertex))
        {
            const TopTools_ListOfShape& edgesOnVertex = vertexToEdgesMap.FindFromKey(vertex);
            for (TopTools_ListIteratorOfListOfShape it(edgesOnVertex); it.More(); it.Next())
            {
                const TopoDS_Edge& edge = TopoDS::Edge(it.Value());
                auto edgeIdOpt = openCascadeGeometryCollection_->findEdgeId(edge);
                if (edgeIdOpt.has_value())
                {
                    connectedEdgeIds.insert(edgeIdOpt.value());
                }
            }
        }

        // Find connected surfaces
        std::set<std::string> connectedSurfaceIds;
        TopTools_IndexedDataMapOfShapeListOfShape vertexToFacesMap;
        TopExp::MapShapesAndAncestors(shape_, TopAbs_VERTEX, TopAbs_FACE, vertexToFacesMap);

        if (vertexToFacesMap.Contains(vertex))
        {
            const TopTools_ListOfShape& facesOnVertex = vertexToFacesMap.FindFromKey(vertex);
            for (TopTools_ListIteratorOfListOfShape it(facesOnVertex); it.More(); it.Next())
            {
                const TopoDS_Face& face = TopoDS::Face(it.Value());
                auto faceIdOpt = openCascadeGeometryCollection_->findSurfaceId(face);
                if (faceIdOpt.has_value())
                {
                    connectedSurfaceIds.insert(faceIdOpt.value());
                }
            }
        }

        corners_.emplace(id, Topology3D::Corner3D(id, connectedEdgeIds, connectedSurfaceIds));
    }
}