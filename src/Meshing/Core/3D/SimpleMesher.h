#pragma once

#include "Meshing/Interfaces/IMesher.h"
#include <string>

namespace Meshing
{
class IQualityController;

// A placeholder/simple mesher implementation that seeds a trivial mesh.
class SimpleMesher : public IMesher
{
public:
    explicit SimpleMesher(const IQualityController* qualityController = nullptr) :
        IMesher(qualityController) {}
    ~SimpleMesher() override = default;

    void generate(MeshingContext3D& context) override;
    std::string getName() const override { return "SimpleMesher"; }
};

} // namespace Meshing
