#include "OpenCascadeGeometryCollection.h"
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>

using namespace Readers;

OpenCascadeGeometryCollection::OpenCascadeGeometryCollection(const TopoDS_Shape& shape) :
    shape_(shape)
{
    buildMaps();
}

const std::map<std::string, TopoDS_Vertex>& OpenCascadeGeometryCollection::getVertexMap() const
{
    return vertexMap_;
}

const std::map<std::string, TopoDS_Edge>& OpenCascadeGeometryCollection::getEdgeMap() const
{
    return edgeMap_;
}

const std::map<std::string, TopoDS_Face>& OpenCascadeGeometryCollection::getFaceMap() const
{
    return faceMap_;
}

std::optional<std::string> OpenCascadeGeometryCollection::findVertexId(const TopoDS_Vertex& vertex) const
{
    for (const auto& [id, vertexVal] : vertexMap_)
    {
        if (vertex.IsSame(vertexVal))
        {
            return id;
        }
    }
    return std::nullopt;
}

std::optional<std::string> OpenCascadeGeometryCollection::findEdgeId(const TopoDS_Edge& edge) const
{
    for (const auto& [id, edgeVal] : edgeMap_)
    {
        if (edge.IsSame(edgeVal))
        {
            return id;
        }
    }
    return std::nullopt;
}

std::optional<std::string> OpenCascadeGeometryCollection::findSurfaceId(const TopoDS_Face& face) const
{
    for (const auto& [id, faceVal] : faceMap_)
    {
        if (face.IsSame(faceVal))
        {
            return id;
        }
    }
    return std::nullopt;
}

void OpenCascadeGeometryCollection::buildMaps()
{
    // Build face map
    TopExp_Explorer faceExplorer(shape_, TopAbs_FACE);
    size_t faceCount = 0;
    while (faceExplorer.More())
    {
        TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
        std::string faceId = "face_" + std::to_string(faceCount);
        faceMap_[faceId] = face;
        faceExplorer.Next();
        faceCount++;
    }

    // Build edge map
    TopTools_IndexedMapOfShape edgeMap;
    TopExp::MapShapes(shape_, TopAbs_EDGE, edgeMap);

    for (int i = 1; i <= edgeMap.Extent(); i++)
    {
        TopoDS_Edge edge = TopoDS::Edge(edgeMap.FindKey(i));
        std::string edgeId = "edge_" + std::to_string(i - 1);
        edgeMap_[edgeId] = edge;
    }

    // Build vertex map
    TopTools_IndexedMapOfShape vertexMap;
    TopExp::MapShapes(shape_, TopAbs_VERTEX, vertexMap);

    for (int i = 1; i <= vertexMap.Extent(); i++)
    {
        TopoDS_Vertex vertex = TopoDS::Vertex(vertexMap.FindKey(i));
        std::string vertexId = "vertex_" + std::to_string(i - 1);
        vertexMap_[vertexId] = vertex;
    }
}