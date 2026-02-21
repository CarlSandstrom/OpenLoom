#pragma once

#include <exception>
#include <string>

namespace OpenLoom
{

class Exception : public std::exception
{
public:
    explicit Exception(
        std::string message,
        std::string location = "") :
        message_(std::move(message)), location_(std::move(location)), fullMessage_(formatMessage())
    {
    }

    virtual ~Exception() noexcept = default;

    const char* what() const noexcept override
    {
        return fullMessage_.c_str();
    }

    const std::string& message() const noexcept
    {
        return message_;
    }

    const std::string& location() const noexcept
    {
        return location_;
    }

    virtual int errorCode() const noexcept
    {
        return 0;
    }

    virtual const char* typeName() const noexcept
    {
        return "OpenLoom::Exception";
    }

protected:
    std::string formatMessage() const
    {
        if (location_.empty())
        {
            return message_;
        }
        return message_ + "\n  at " + location_;
    }

private:
    std::string message_;
    std::string location_;
    std::string fullMessage_;
};

} // namespace OpenLoom

#define OPENLOOM_THROW(ExceptionType, message) \
    throw ExceptionType(message, std::string(__FILE__) + ":" + std::to_string(__LINE__))

#define OPENLOOM_THROW_CODE(ExceptionType, code, message) \
    throw ExceptionType(code, message, std::string(__FILE__) + ":" + std::to_string(__LINE__))

#define OPENLOOM_REQUIRE(condition, ExceptionType, message) \
    do                                                   \
    {                                                    \
        if (!(condition))                                \
        {                                                \
            OPENLOOM_THROW(ExceptionType, message);         \
        }                                                \
    } while (0)
