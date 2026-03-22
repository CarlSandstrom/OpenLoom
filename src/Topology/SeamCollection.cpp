#include "SeamCollection.h"
#include "Common/Exceptions/GeometryException.h"

namespace Topology3D
{

void SeamCollection::addPair(const std::string& originalEdgeId,
                              const std::string& twinEdgeId,
                              SeamDirection direction,
                              std::array<double, 2> twinStartUV,
                              std::array<double, 2> twinEndUV)
{
    twinToOriginal_[twinEdgeId] = originalEdgeId;
    twinToDirection_[twinEdgeId] = direction;
    twinToStartUV_[twinEdgeId] = twinStartUV;
    twinToEndUV_[twinEdgeId] = twinEndUV;
}

bool SeamCollection::isSeamTwin(const std::string& edgeId) const
{
    return twinToOriginal_.count(edgeId) > 0;
}

const std::string& SeamCollection::getOriginalEdgeId(const std::string& twinEdgeId) const
{
    auto it = twinToOriginal_.find(twinEdgeId);
    if (it == twinToOriginal_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("SeamTwinEdge", twinEdgeId);
    }
    return it->second;
}

SeamCollection::SeamDirection SeamCollection::getSeamDirection(const std::string& twinEdgeId) const
{
    auto it = twinToDirection_.find(twinEdgeId);
    if (it == twinToDirection_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("SeamTwinEdge", twinEdgeId);
    }
    return it->second;
}

std::array<double, 2> SeamCollection::getTwinStartUV(const std::string& twinEdgeId) const
{
    auto it = twinToStartUV_.find(twinEdgeId);
    if (it == twinToStartUV_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("SeamTwinEdge", twinEdgeId);
    }
    return it->second;
}

std::array<double, 2> SeamCollection::getTwinEndUV(const std::string& twinEdgeId) const
{
    auto it = twinToEndUV_.find(twinEdgeId);
    if (it == twinToEndUV_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("SeamTwinEdge", twinEdgeId);
    }
    return it->second;
}

std::vector<std::string> SeamCollection::getSeamTwinEdgeIds() const
{
    std::vector<std::string> ids;
    ids.reserve(twinToOriginal_.size());
    for (const auto& [twinId, _] : twinToOriginal_)
    {
        ids.push_back(twinId);
    }
    return ids;
}

bool SeamCollection::empty() const
{
    return twinToOriginal_.empty();
}

} // namespace Topology3D
