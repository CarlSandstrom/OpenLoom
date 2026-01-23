#pragma once

#include "Common/Exceptions/Exception.h"
#include <string>
#include <vector>

namespace cMesh
{

class MeshException : public Exception
{
public:
    enum class ErrorCode
    {
        GENERATION_FAILED = 2000,
        VERIFICATION_FAILED,
        DEGENERATE_TRIANGLE,
        CONSTRAINT_VIOLATION,
        MAX_ITERATIONS_REACHED,
        INVALID_TOPOLOGY,
        NODE_NOT_FOUND,
        ELEMENT_NOT_FOUND,
        INVALID_OPERATION
    };

    MeshException(
        ErrorCode code,
        std::string message,
        std::string location = "") :
        Exception(std::move(message), std::move(location)), code_(code)
    {
    }

    ErrorCode code() const noexcept
    {
        return code_;
    }

    int errorCode() const noexcept override
    {
        return static_cast<int>(code_);
    }

    const char* typeName() const noexcept override
    {
        return "cMesh::MeshException";
    }

private:
    ErrorCode code_;
};

class MeshVerificationException : public MeshException
{
public:
    MeshVerificationException(
        std::string message,
        std::vector<std::string> errors,
        std::string location = "") :
        MeshException(
            ErrorCode::VERIFICATION_FAILED,
            std::move(message),
            std::move(location)),
        errors_(std::move(errors))
    {
    }

    explicit MeshVerificationException(
        std::string message,
        std::string location = "") :
        MeshException(
            ErrorCode::VERIFICATION_FAILED,
            std::move(message),
            std::move(location)),
        errors_()
    {
    }

    const std::vector<std::string>& getErrors() const
    {
        return errors_;
    }

private:
    std::vector<std::string> errors_;
};

class MaxIterationsException : public MeshException
{
public:
    MaxIterationsException(
        std::string operation,
        int maxIterations,
        std::string location = "") :
        MeshException(
            ErrorCode::MAX_ITERATIONS_REACHED,
            operation + " failed to converge after " + std::to_string(maxIterations) + " iterations",
            std::move(location)),
        operation_(std::move(operation)), maxIterations_(maxIterations)
    {
    }

    const std::string& getOperation() const
    {
        return operation_;
    }

    int getMaxIterations() const
    {
        return maxIterations_;
    }

private:
    std::string operation_;
    int maxIterations_;
};

class MeshEntityNotFoundException : public MeshException
{
public:
    MeshEntityNotFoundException(
        std::string entityType,
        size_t entityId,
        std::string location = "") :
        MeshException(
            ErrorCode::NODE_NOT_FOUND,
            entityType + " with ID " + std::to_string(entityId) + " not found",
            std::move(location)),
        entityType_(std::move(entityType)), entityId_(entityId)
    {
    }

    const std::string& getEntityType() const
    {
        return entityType_;
    }

    size_t getEntityId() const
    {
        return entityId_;
    }

private:
    std::string entityType_;
    size_t entityId_;
};

} // namespace cMesh

#define CMESH_THROW_MESH(code, message) \
    CMESH_THROW_CODE(cMesh::MeshException, cMesh::MeshException::ErrorCode::code, message)

#define CMESH_THROW_VERIFICATION_FAILED(message, errors)    \
    throw cMesh::MeshVerificationException(message, errors, \
                                           std::string(__FILE__) + ":" + std::to_string(__LINE__))

#define CMESH_THROW_MAX_ITERATIONS(operation, maxIter)      \
    throw cMesh::MaxIterationsException(operation, maxIter, \
                                        std::string(__FILE__) + ":" + std::to_string(__LINE__))
