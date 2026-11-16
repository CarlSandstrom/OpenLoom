#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace Meshing
{

/**
 * @brief A strongly typed, dense, zero-based ID.
 *
 * Wraps a size_t so that IDs of different entities (nodes, elements, curve
 * segments) cannot be mixed up: there is no implicit conversion to or from a
 * raw number, and two Identifier types with different tags are unrelated.
 * Value 0 is a valid ID, so the default constructor is not a null state; use
 * INVALID for "no ID".
 */
template <typename Tag>
class Identifier
{
public:
    constexpr Identifier() = default;
    explicit constexpr Identifier(size_t value) :
        value_(value)
    {
    }

    constexpr size_t value() const { return value_; }

    constexpr auto operator<=>(const Identifier&) const = default;
    constexpr bool operator==(const Identifier&) const = default;

    static const Identifier INVALID;

private:
    size_t value_ = 0;
};

template <typename Tag>
inline constexpr Identifier<Tag> Identifier<Tag>::INVALID{SIZE_MAX};

} // namespace Meshing

template <typename Tag>
struct std::hash<Meshing::Identifier<Tag>>
{
    size_t operator()(const Meshing::Identifier<Tag>& identifier) const
    {
        return std::hash<size_t>{}(identifier.value());
    }
};
