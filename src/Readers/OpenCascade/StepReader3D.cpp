#include "StepReader3D.h"

#include "Common/Exceptions/GeometryException.h"
#include <STEPControl_Reader.hxx>
#include <spdlog/spdlog.h>

using namespace Readers;

StepReader3D::StepReader3D(const std::string& filePath)
{
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(filePath.c_str());

    if (status != IFSelect_RetDone)
    {
        OPENLOOM_THROW_CODE(OpenLoom::GeometryException,
                            OpenLoom::GeometryException::ErrorCode::INVALID_GEOMETRY,
                            "Failed to read STEP file: " + filePath);
    }

    reader.TransferRoots();
    shape_ = reader.OneShape();

    spdlog::info("STEP file loaded: {}", filePath);
}

const TopoDS_Shape& StepReader3D::getShape() const
{
    return shape_;
}
