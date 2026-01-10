#pragma once

#include "Common/Exceptions/Exception.h"
#include <string>

namespace cMesh {

class TopologyException : public Exception {
public:
    enum class ErrorCode {
        ENTITY_NOT_FOUND = 4000,
        INVALID_CONNECTION,
        INCONSISTENT_TOPOLOGY
    };

    TopologyException(
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
        return "cMesh::TopologyException";
    }

private:
    ErrorCode code_;
};

} // namespace cMesh

#define CMESH_THROW_TOPOLOGY(code, message) \
    CMESH_THROW_CODE(cMesh::TopologyException, cMesh::TopologyException::ErrorCode::code, message)
