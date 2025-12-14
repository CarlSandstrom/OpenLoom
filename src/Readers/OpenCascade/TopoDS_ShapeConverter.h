#pragma once

#include "Geometry/Base/GeometryCollection3D.h"
#include "OpenCascadeGeometryCollection.h"
#include "Topology/Corner3D.h"
#include "Topology/Edge3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"
#include <TopoDS_Shape.hxx>
#include <memory>
#include <unordered_map>

namespace Readers
{

class TopoDS_ShapeConverter
{
public:
    TopoDS_ShapeConverter(const TopoDS_Shape& shape);

    const Topology3D::Topology3D& getTopology() const;
    const Geometry3D::GeometryCollection3D& getGeometryCollection() const;

private:
    void buildTopology();
    void buildGeometryCollection();
    void createSurfaces();
    void createEdges();
    void createCorners();

private:
    const TopoDS_Shape& shape_;
    std::unique_ptr<OpenCascadeGeometryCollection> openCascadeGeometryCollection_;

    std::unordered_map<std::string, Topology3D::Corner3D> corners_;
    std::unordered_map<std::string, Topology3D::Edge3D> edges_;
    std::unordered_map<std::string, Topology3D::Surface3D> surfaces_;
    std::unique_ptr<Topology3D::Topology3D> topology_;

    std::unique_ptr<Geometry3D::GeometryCollection3D> geometryCollection_;
};
} // namespace Readers