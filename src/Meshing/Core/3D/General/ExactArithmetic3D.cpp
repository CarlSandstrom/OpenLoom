#include "Meshing/Core/3D/General/ExactArithmetic3D.h"

#include <algorithm>
#include <cmath>

namespace Meshing
{

ExactArithmetic3D::TwoResult ExactArithmetic3D::twoSum(double a, double b)
{
    double sum = a + b;
    double bVirtual = sum - a;
    double aVirtual = sum - bVirtual;
    double bRoundoff = b - bVirtual;
    double aRoundoff = a - aVirtual;
    return {sum, aRoundoff + bRoundoff};
}

ExactArithmetic3D::TwoResult ExactArithmetic3D::twoProduct(double a, double b)
{
    double product = a * b;
    double error = std::fma(a, b, -product);
    return {product, error};
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionFromDouble(double a)
{
    return a == 0.0 ? Expansion{} : Expansion{a};
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionFromDifference(double a, double b)
{
    auto difference = twoSum(a, -b);
    return difference.error == 0.0 ? Expansion{difference.value} : Expansion{difference.error, difference.value};
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionAdd(const Expansion& first, const Expansion& second)
{
    Expansion merged(first.size() + second.size());
    std::merge(first.begin(), first.end(), second.begin(), second.end(), merged.begin(),
               [](double left, double right)
               { return std::abs(left) < std::abs(right); });

    if (merged.empty())
        return {};

    Expansion result;
    result.reserve(merged.size());
    double carry = merged[0];
    for (size_t k = 1; k < merged.size(); ++k)
    {
        auto sum = twoSum(carry, merged[k]);
        if (sum.error != 0.0)
            result.push_back(sum.error);
        carry = sum.value;
    }
    if (carry != 0.0)
        result.push_back(carry);
    return result;
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionNegate(const Expansion& expansion)
{
    Expansion result = expansion;
    for (double& component : result)
        component = -component;
    return result;
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionSub(const Expansion& first, const Expansion& second)
{
    return expansionAdd(first, expansionNegate(second));
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionScale(const Expansion& expansion, double scalar)
{
    Expansion result;
    for (double component : expansion)
    {
        auto product = twoProduct(component, scalar);
        result = expansionAdd(result, expansionFromDouble(product.error));
        result = expansionAdd(result, expansionFromDouble(product.value));
    }
    return result;
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionMul(const Expansion& first, const Expansion& second)
{
    Expansion result;
    for (double component : second)
        result = expansionAdd(result, expansionScale(first, component));
    return result;
}

int ExactArithmetic3D::expansionSign(const Expansion& expansion)
{
    if (expansion.empty())
        return 0;
    return expansion.back() > 0.0 ? 1 : -1;
}

ExactArithmetic3D::Expansion ExactArithmetic3D::det2x2(const Expansion& a, const Expansion& b, const Expansion& c,
                                                       const Expansion& d)
{
    return expansionSub(expansionMul(a, d), expansionMul(b, c));
}

ExactArithmetic3D::Expansion ExactArithmetic3D::det3x3(const Expansion m[3][3])
{
    Expansion c0 = det2x2(m[1][1], m[1][2], m[2][1], m[2][2]);
    Expansion c1 = det2x2(m[1][0], m[1][2], m[2][0], m[2][2]);
    Expansion c2 = det2x2(m[1][0], m[1][1], m[2][0], m[2][1]);
    return expansionSub(expansionAdd(expansionMul(m[0][0], c0), expansionMul(m[0][2], c2)), expansionMul(m[0][1], c1));
}

ExactArithmetic3D::Expansion ExactArithmetic3D::squaredNormRelative(const Expansion& dx, const Expansion& dy,
                                                                    const Expansion& dz)
{
    return expansionAdd(expansionAdd(expansionMul(dx, dx), expansionMul(dy, dy)), expansionMul(dz, dz));
}

} // namespace Meshing
