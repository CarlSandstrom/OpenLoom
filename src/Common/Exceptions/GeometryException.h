#pragma once

#include "Common/Exceptions/Exception.h"
#include <string>

namespace cMesh {

class GeometryException : public Exception {
public:
    enum class ErrorCode {
        ENTITY_NOT_FOUND = 1000,
        INVALID_GEOMETRY,
        NULL_POINTER,
        EMPTY_COLLECTION,
        WIRE_BUILDING_FAILED,
        EDGE_LOOP_INVALID,
        PARAMETER_OUT_OF_RANGE,
        DEGENERATE_GEOMETRY
    };

    explicit GeometryException(
        std::string message,
        std::string location = ""
    )
        : Exception(std::move(message), std::move(location))
        , code_(ErrorCode::INVALID_GEOMETRY)
    {}

    GeometryException(
        ErrorCode code,
        std::string message,
        std::string location = ""
    )
        : Exception(std::move(message), std::move(location))
        , code_(code)
    {}

    ErrorCode code() const noexcept {
        return code_;
    }

    int errorCode() const noexcept override {
        return static_cast<int>(code_);
    }

    const char* typeName() const noexcept override {
        return "cMesh::GeometryException";
    }

private:
    ErrorCode code_;
};

class EntityNotFoundException : public GeometryException {
public:
    EntityNotFoundException(
        std::string entityType,
        std::string entityId,
        std::string location = ""
    )
        : GeometryException(
            ErrorCode::ENTITY_NOT_FOUND,
            entityType + " with ID '" + entityId + "' not found",
            std::move(location))
        , entityType_(std::move(entityType))
        , entityId_(std::move(entityId))
    {}

    const std::string& getEntityType() const {
        return entityType_;
    }

    const std::string& getEntityId() const {
        return entityId_;
    }

private:
    std::string entityType_;
    std::string entityId_;
};

class NullGeometryException : public GeometryException {
public:
    explicit NullGeometryException(
        std::string what,
        std::string location = ""
    )
        : GeometryException(
            ErrorCode::NULL_POINTER,
            "Null geometry pointer: " + what,
            std::move(location))
    {}
};

} // namespace cMesh

#define CMESH_THROW_GEOMETRY(message) \
    CMESH_THROW(cMesh::GeometryException, message)

#define CMESH_THROW_ENTITY_NOT_FOUND(entityType, entityId) \
    throw cMesh::EntityNotFoundException(entityType, entityId, \
        std::string(__FILE__) + ":" + std::to_string(__LINE__))

#define CMESH_REQUIRE_NOT_NULL(ptr, name) \
    CMESH_REQUIRE(ptr != nullptr, cMesh::NullGeometryException, name)
