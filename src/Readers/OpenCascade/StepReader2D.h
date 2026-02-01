#pragma once

#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Topology2D/Topology2D.h"
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace Readers
{

class StepReader2D
{
public:
    explicit StepReader2D(const std::string& filePath);

    const Topology2D::Topology2D& getTopology() const;
    const Geometry2D::GeometryCollection2D& getGeometry() const;

    std::unique_ptr<Topology2D::Topology2D> takeTopology();
    std::unique_ptr<Geometry2D::GeometryCollection2D> takeGeometry();

private:
    void loadStep(const std::string& filePath);
    void extractFace();
    void buildTopologyAndGeometry();
    void processWire(const TopoDS_Wire& wire,
                     const TopoDS_Face& face,
                     std::vector<std::string>& edgeLoop);

    std::string findOrCreateVertex(const TopoDS_Vertex& vertex,
                                   const TopoDS_Face& face);

private:
    TopoDS_Shape shape_;
    TopoDS_Face face_;
    std::unique_ptr<Topology2D::Topology2D> topology_;
    std::unique_ptr<Geometry2D::GeometryCollection2D> geometry_;

    // Accumulated during wire processing, assembled into Topology2D at the end
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges_;
    std::unordered_map<std::string, std::set<std::string>> vertexEdgeConnectivity_;

    // Vertex deduplication: stores (TopoDS_Vertex, vertexId) pairs
    std::vector<std::pair<TopoDS_Vertex, std::string>> knownVertices_;
    int edgeCounter_ = 0;
    int vertexCounter_ = 0;
};

} // namespace Readers
