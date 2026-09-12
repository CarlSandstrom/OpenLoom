#include "Meshing/Core/3D/General/ExactArithmetic3D.h"

#include <cmath>

namespace Meshing
{

ExactArithmetic3D::TwoResult ExactArithmetic3D::twoSum(double a, double b)
{
    const double sum = a + b;
    const double bVirtual = sum - a;
    const double aVirtual = sum - bVirtual;
    const double bRoundoff = b - bVirtual;
    const double aRoundoff = a - aVirtual;
    return {sum, aRoundoff + bRoundoff};
}

ExactArithmetic3D::TwoResult ExactArithmetic3D::twoProduct(double a, double b)
{
    const double product = a * b;
    const double error = std::fma(a, b, -product);
    return {product, error};
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionFromDouble(double a)
{
    return a == 0.0 ? Expansion{} : Expansion{a};
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionFromDifference(double a, double b)
{
    const TwoResult r = twoSum(a, -b);
    return r.error == 0.0 ? Expansion{r.value} : Expansion{r.error, r.value};
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionAdd(const Expansion& e, const Expansion& f)
{
    std::vector<double> merged;
    merged.reserve(e.size() + f.size());
    size_t i = 0;
    size_t j = 0;
    while (i < e.size() && j < f.size())
    {
        if (std::abs(e[i]) <= std::abs(f[j]))
            merged.push_back(e[i++]);
        else
            merged.push_back(f[j++]);
    }
    while (i < e.size())
        merged.push_back(e[i++]);
    while (j < f.size())
        merged.push_back(f[j++]);

    if (merged.empty())
        return {};

    Expansion result;
    result.reserve(merged.size());
    double carry = merged[0];
    for (size_t k = 1; k < merged.size(); ++k)
    {
        const TwoResult r = twoSum(carry, merged[k]);
        if (r.error != 0.0)
            result.push_back(r.error);
        carry = r.value;
    }
    if (carry != 0.0)
        result.push_back(carry);
    return result;
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionNegate(const Expansion& e)
{
    Expansion result = e;
    for (double& component : result)
        component = -component;
    return result;
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionSub(const Expansion& e, const Expansion& f)
{
    return expansionAdd(e, expansionNegate(f));
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionScale(const Expansion& e, double b)
{
    Expansion result;
    for (double component : e)
    {
        const TwoResult p = twoProduct(component, b);
        result = expansionAdd(result, expansionFromDouble(p.error));
        result = expansionAdd(result, expansionFromDouble(p.value));
    }
    return result;
}

ExactArithmetic3D::Expansion ExactArithmetic3D::expansionMul(const Expansion& e, const Expansion& f)
{
    Expansion result;
    for (double component : f)
        result = expansionAdd(result, expansionScale(e, component));
    return result;
}

int ExactArithmetic3D::expansionSign(const Expansion& e)
{
    if (e.empty())
        return 0;
    return e.back() > 0.0 ? 1 : -1;
}

ExactArithmetic3D::Expansion ExactArithmetic3D::det2x2(const Expansion& a, const Expansion& b, const Expansion& c,
                                                        const Expansion& d)
{
    return expansionSub(expansionMul(a, d), expansionMul(b, c));
}

ExactArithmetic3D::Expansion ExactArithmetic3D::det3x3(const Expansion m[3][3])
{
    const Expansion c0 = det2x2(m[1][1], m[1][2], m[2][1], m[2][2]);
    const Expansion c1 = det2x2(m[1][0], m[1][2], m[2][0], m[2][2]);
    const Expansion c2 = det2x2(m[1][0], m[1][1], m[2][0], m[2][1]);
    return expansionSub(expansionAdd(expansionMul(m[0][0], c0), expansionMul(m[0][2], c2)), expansionMul(m[0][1], c1));
}

ExactArithmetic3D::Expansion ExactArithmetic3D::det4x4(const Expansion m[4][4])
{
    // Cofactor expansion along the first column: det = sum_i (-1)^i * m[i][0] * Minor_i,
    // where Minor_i is the 3x3 determinant of the other 3 rows' columns 1-3.
    Expansion determinant;
    for (int skipRow = 0; skipRow < 4; ++skipRow)
    {
        Expansion sub[3][3];
        int r = 0;
        for (int row = 0; row < 4; ++row)
        {
            if (row == skipRow)
                continue;
            sub[r][0] = m[row][1];
            sub[r][1] = m[row][2];
            sub[r][2] = m[row][3];
            ++r;
        }
        Expansion term = expansionMul(m[skipRow][0], det3x3(sub));
        if (skipRow % 2 != 0)
            term = expansionNegate(term);
        determinant = expansionAdd(determinant, term);
    }
    return determinant;
}

ExactArithmetic3D::Expansion ExactArithmetic3D::squaredNormRelative(const Expansion& dx, const Expansion& dy,
                                                                     const Expansion& dz)
{
    return expansionAdd(expansionAdd(expansionMul(dx, dx), expansionMul(dy, dy)), expansionMul(dz, dz));
}

} // namespace Meshing
