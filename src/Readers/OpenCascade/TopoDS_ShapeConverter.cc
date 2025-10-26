#include "TopoDS_ShapeConverter.h"
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <optional>
#include <set>
using namespace Readers;

TopoDS_ShapeConverter::TopoDS_ShapeConverter(const TopoDS_Shape& shape) :
    shape_(shape),
    geometryCollection_(std::make_unique<OpenCascadeGeometryCollection>(shape))
{
    buildTopology();
}

const Topology::Topology& TopoDS_ShapeConverter::getTopology() const
{
    return *topology_;
}

void TopoDS_ShapeConverter::buildTopology()
{
    createSurfaces();
    createEdges();
    createCorners();

    topology_ = std::make_unique<Topology::Topology>(std::unordered_map<std::string, Topology::Surface>{},
                                                     std::unordered_map<std::string, Topology::Edge>{},
                                                     std::unordered_map<std::string, Topology::Corner>{});
}

void TopoDS_ShapeConverter::createSurfaces()
{
    const auto& faceMap = geometryCollection_->getFaceMap();

    // Map edges to their adjacent faces. This is used for finding adjacent surfaces.
    TopTools_IndexedDataMapOfShapeListOfShape edgeToFacesMap;
    TopExp::MapShapesAndAncestors(shape_, TopAbs_EDGE, TopAbs_FACE, edgeToFacesMap);

    for (auto& [id, face] : faceMap)
    {
        std::vector<std::string> boundaryEdgeIds;
        std::vector<std::string> cornerIds;
        std::vector<std::string> adjacentSurfaceIds;

        // Find all edges of the face
        TopTools_IndexedMapOfShape edgeMap;
        TopExp::MapShapes(face, TopAbs_EDGE, edgeMap);

        for (size_t i = 1; i <= edgeMap.Extent(); ++i)
        {
            const TopoDS_Edge& edge = TopoDS::Edge(edgeMap(i));
            auto edgeId = geometryCollection_->findEdgeId(edge);
            if (edgeId.has_value())
            {
                boundaryEdgeIds.push_back(edgeId.value());
            }
        }

        // Find all vertices of the face
        TopTools_IndexedMapOfShape vertexMap;
        TopExp::MapShapes(face, TopAbs_VERTEX, vertexMap);

        for (size_t i = 1; i <= vertexMap.Extent(); ++i)
        {
            const TopoDS_Vertex& vertex = TopoDS::Vertex(vertexMap(i));
            auto vertexId = geometryCollection_->findVertexId(vertex);
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
                        auto adjacentFaceId = geometryCollection_->findSurfaceId(adjacentFace);
                        if (adjacentFaceId.has_value())
                        {
                            adjacentSurfaceIds.push_back(adjacentFaceId.value());
                        }
                    }
                }
            }
        }

        surfaces_.emplace(id, Topology::Surface(id, boundaryEdgeIds, cornerIds, adjacentSurfaceIds, {}));
    }
}

void TopoDS_ShapeConverter::createEdges()
{
    for (auto& [id, edge] : geometryCollection_->getEdgeMap())
    {
        // Get start and end vertices
        TopoDS_Vertex startVertex, endVertex;
        TopExp::Vertices(edge, startVertex, endVertex);

        auto startCornerIdOpt = geometryCollection_->findVertexId(startVertex);
        auto endCornerIdOpt = geometryCollection_->findVertexId(endVertex);

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
                auto faceIdOpt = geometryCollection_->findSurfaceId(face);
                if (faceIdOpt.has_value())
                {
                    adjacentSurfaceIds.push_back(faceIdOpt.value());
                }
            }
        }

        edges_.emplace(id, Topology::Edge(id, startCornerId, endCornerId, adjacentSurfaceIds));
    }
}

void TopoDS_ShapeConverter::createCorners()
{
    for (auto& [id, vertex] : geometryCollection_->getVertexMap())
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
                auto edgeIdOpt = geometryCollection_->findEdgeId(edge);
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
                auto faceIdOpt = geometryCollection_->findSurfaceId(face);
                if (faceIdOpt.has_value())
                {
                    connectedSurfaceIds.insert(faceIdOpt.value());
                }
            }
        }

        corners_.emplace(id, Topology::Corner(id, connectedEdgeIds, connectedSurfaceIds));
    }
}