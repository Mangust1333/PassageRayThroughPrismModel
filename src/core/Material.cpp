#include <core/Material.hpp>
#include <cmath>

double core::Material::refractiveIndex(double lambda_nm) const {
    if (this->special_value > 0) {
        return this->special_value;
    }

    double λ = lambda_nm / 1000.0;
    double n2 = 1.0;
    for (int i = 0; i < 3; ++i) {
        n2 += (B[i] * λ * λ) / (λ * λ - C[i]);
    }

    return std::sqrt(std::abs(n2));
}