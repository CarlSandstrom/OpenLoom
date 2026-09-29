#pragma once

#include <TopoDS_Shape.hxx>
#include <string>

namespace Readers
{

/**
 * @brief Loads a 3D solid from a STEP file into a TopoDS_Shape.
 *
 * Extracting geometry and topology from the shape is TopoDS_ShapeConverter's
 * job -- this class only gets the shape off disk, the same way StepReader2D
 * does for a 2D sketch.
 */
class StepReader3D
{
public:
    explicit StepReader3D(const std::string& filePath);

    const TopoDS_Shape& getShape() const;

private:
    TopoDS_Shape shape_;
};

} // namespace Readers
