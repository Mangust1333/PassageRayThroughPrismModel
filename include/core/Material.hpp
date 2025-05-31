#pragma once
#include <array>

namespace core {

    struct Material {
        std::array<double, 3> B;  // коэффициенты Бриллюэна
        std::array<double, 3> C;  // коэффициенты Лоренца
        double special_value;     // для «зеркала» (если n_out == специальное значение)

        double refractiveIndex(double lambda_nm) const;
    };

} // namespace core
