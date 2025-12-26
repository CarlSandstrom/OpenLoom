#pragma once

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <map>
#include <optional>
#include <string>

namespace Readers
{

class OpenCascadeGeometryCollection
{
public:
    explicit OpenCascadeGeometryCollection(const TopoDS_Shape& shape);

    // Getters
    const std::map<std::string, TopoDS_Vertex>& getVertexMap() const;
    const std::map<std::string, TopoDS_Edge>& getEdgeMap() const;
    const std::map<std::string, TopoDS_Face>& getFaceMap() const;

    // Methods for finding IDs of objects
    std::optional<std::string> findVertexId(const TopoDS_Vertex& vertex) const;
    std::optional<std::string> findEdgeId(const TopoDS_Edge& edge) const;
    std::optional<std::string> findSurfaceId(const TopoDS_Face& face) const;

private:
    void buildMaps();

private:
    const TopoDS_Shape& shape_;

    std::map<std::string, TopoDS_Vertex> vertexMap_;
    std::map<std::string, TopoDS_Edge> edgeMap_;
    std::map<std::string, TopoDS_Face> faceMap_;
};

} // namespace Readers