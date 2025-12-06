#include "Meshing/Core/Delaunay3D.h"

#include "Export/VtkExporter.h"
#include "Meshing/Data/TetrahedralElement.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace Meshing
{
Delaunay3D::Delaunay3D(Meshing::MeshingContext& context) :
    computer_(context.getMeshData()),
    context_(context),
    meshData_(context.getMeshData()),
    operations_(context.getOperations())
{
}

void Delaunay3D::initialize(const std::vector<Point3D>& points)
{
    superNodeIds_.clear();
    activeTetrahedra_.clear();

    if (points.size() < 4)
    {
        throw std::runtime_error("Need at least 4 points for 3D triangulation");
    }

    createSuperTetrahedron(points);

    Export::VtkExporter exporter;
    size_t iteration = 0;
    for (const auto& p : points)
    {
        insertVertex(p);

        std::ostringstream fileName;
        // Dump incremental mesh snapshots to aid debugging of vertex insertions
        fileName << "mesh_" << std::setw(3) << std::setfill('0') << iteration << ".vtu";
        exporter.writeVtu(meshData_, fileName.str());
        ++iteration;
    }

    removeSuperTetrahedron();
}

void Delaunay3D::insertVertex(const Point3D& point)
{
    const size_t nodeId = operations_.addNode(point);

    SPDLOG_INFO("Inserting vertex at ({}, {}, {}) as node ID {}",
                point.x(), point.y(), point.z(), nodeId);

    const std::vector<size_t> conflicting = findConflictingTetrahedra(point);
    if (conflicting.empty())
    {
        throw std::runtime_error("No conflicting tetrahedra found - point outside mesh?");
    }

    const std::vector<std::array<size_t, 3>> boundary = findCavityBoundary(conflicting);

    for (size_t elementId : conflicting)
    {
        operations_.removeElement(elementId);
        activeTetrahedra_.erase(elementId);
    }

    retriangulate(nodeId, boundary);
}

bool Delaunay3D::isElementActive(size_t elementId) const
{
    return activeTetrahedra_.find(elementId) != activeTetrahedra_.end();
}

const TetrahedralElement* Delaunay3D::getTetrahedralElement(size_t elementId) const
{
    const auto* element = meshData_.getElement(elementId);
    return dynamic_cast<const TetrahedralElement*>(element);
}

std::vector<size_t> Delaunay3D::findConflictingTetrahedra(const Point3D& p) const
{
    std::vector<size_t> conflicting;
    conflicting.reserve(activeTetrahedra_.size());

    for (size_t elementId : activeTetrahedra_)
    {
        const auto* element = getTetrahedralElement(elementId);
        if (element == nullptr)
        {
            continue;
        }

        if (computer_.getIsPointInsideCircumscribingSphere(*element, p))
        {
            conflicting.push_back(elementId);
        }
    }

    return conflicting;
}

std::vector<std::array<size_t, 3>> Delaunay3D::findCavityBoundary(const std::vector<size_t>& conflicting) const
{
    std::map<std::array<size_t, 3>, int> faceCount;
    std::map<std::array<size_t, 3>, std::array<size_t, 3>> faceLookup;

    for (size_t elementId : conflicting)
    {
        const auto* element = getTetrahedralElement(elementId);
        if (element == nullptr)
        {
            continue;
        }

        for (const auto& face : element->getFaces())
        {
            auto sortedFace = face;
            std::sort(sortedFace.begin(), sortedFace.end());
            faceCount[sortedFace]++;
            faceLookup.emplace(sortedFace, face);
        }
    }

    std::vector<std::array<size_t, 3>> boundary;
    boundary.reserve(faceCount.size());

    for (const auto& [face, count] : faceCount)
    {
        if (count == 1)
        {
            boundary.push_back(faceLookup.at(face));
        }
    }

    return boundary;
}

void Delaunay3D::retriangulate(size_t vertexNodeId, const std::vector<std::array<size_t, 3>>& boundary)
{
    for (const auto& face : boundary)
    {
        std::array<size_t, 4> nodeIds = {vertexNodeId, face[0], face[1], face[2]};
        auto element = std::make_unique<TetrahedralElement>(nodeIds);
        const size_t elementId = operations_.addElement(std::move(element));
        activeTetrahedra_.insert(elementId);
    }
}

void Delaunay3D::createSuperTetrahedron(const std::vector<Point3D>& points)
{
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();

    for (const auto& p : points)
    {
        minX = std::min(minX, p.x());
        minY = std::min(minY, p.y());
        minZ = std::min(minZ, p.z());
        maxX = std::max(maxX, p.x());
        maxY = std::max(maxY, p.y());
        maxZ = std::max(maxZ, p.z());
    }

    const double dx = maxX - minX;
    const double dy = maxY - minY;
    const double dz = maxZ - minZ;
    const double dmax = std::max({dx, dy, dz});

    const double midX = (minX + maxX) * 0.5;
    const double midY = (minY + maxY) * 0.5;
    const double midZ = (minZ + maxZ) * 0.5;

    const double scale = 10.0 * dmax;

    superNodeIds_.push_back(operations_.addNode(Point3D(midX - scale, midY - scale, midZ - scale)));
    superNodeIds_.push_back(operations_.addNode(Point3D(midX + scale, midY - scale, midZ - scale)));
    superNodeIds_.push_back(operations_.addNode(Point3D(midX, midY + scale, midZ - scale)));
    superNodeIds_.push_back(operations_.addNode(Point3D(midX, midY, midZ + scale)));

    std::array<size_t, 4> nodeIds = {superNodeIds_[0], superNodeIds_[1], superNodeIds_[2], superNodeIds_[3]};
    auto element = std::make_unique<TetrahedralElement>(nodeIds);
    const size_t elementId = operations_.addElement(std::move(element));
    activeTetrahedra_.insert(elementId);
}

void Delaunay3D::removeSuperTetrahedron()
{
    if (superNodeIds_.empty())
    {
        return;
    }

    const std::unordered_set<size_t> superNodeSet(superNodeIds_.begin(), superNodeIds_.end());

    std::vector<size_t> elementsToRemove;
    for (size_t elementId : activeTetrahedra_)
    {
        const auto* element = getTetrahedralElement(elementId);
        if (element == nullptr)
        {
            continue;
        }

        for (size_t nodeId : element->getNodeIds())
        {
            if (superNodeSet.count(nodeId) > 0)
            {
                elementsToRemove.push_back(elementId);
                break;
            }
        }
    }

    for (size_t elementId : elementsToRemove)
    {
        operations_.removeElement(elementId);
        activeTetrahedra_.erase(elementId);
    }

    for (size_t nodeId : superNodeIds_)
    {
        operations_.removeNode(nodeId);
    }

    superNodeIds_.clear();
}

bool Delaunay3D::isDelaunay() const
{
    for (size_t elementId : activeTetrahedra_)
    {
        const auto* element = getTetrahedralElement(elementId);
        if (element == nullptr)
        {
            continue;
        }

        const auto sphere = computer_.getCircumscribingSphere(*element);
        if (!sphere)
        {
            continue;
        }

        for (const auto& [nodeId, nodePtr] : meshData_.getNodes())
        {
            if (element->hasNode(nodeId))
            {
                continue;
            }

            if (computer_.getIsPointInsideCircumscribingSphere(*sphere, nodePtr->getCoordinates()))
            {
                return false;
            }
        }
    }

    return true;
}

} // namespace Meshing
