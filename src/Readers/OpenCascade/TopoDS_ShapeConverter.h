#pragma once

#include "../../Geometry/Base/GeometryCollection.h"
#include "Corner.h"
#include "Edge.h"
#include "OpenCascadeGeometryCollection.h"
#include "Surface.h"
#include "Topology.h"
#include <TopoDS_Shape.hxx>
#include <memory>
#include <unordered_map>

namespace Readers
{

class TopoDS_ShapeConverter
{
public:
    TopoDS_ShapeConverter(const TopoDS_Shape& shape);

    const Topology::Topology& getTopology() const;
    const Geometry::GeometryCollection& getGeometryCollection() const;

private:
    void buildTopology();
    void buildGeometryCollection();
    void createSurfaces();
    void createEdges();
    void createCorners();

private:
    const TopoDS_Shape& shape_;
    std::unique_ptr<OpenCascadeGeometryCollection> openCascadeGeometryCollection_;

    std::unordered_map<std::string, Topology::Corner> corners_;
    std::unordered_map<std::string, Topology::Edge> edges_;
    std::unordered_map<std::string, Topology::Surface> surfaces_;
    std::unique_ptr<Topology::Topology> topology_;

    std::unique_ptr<Geometry::GeometryCollection> geometryCollection_;
};
} // namespace Readers