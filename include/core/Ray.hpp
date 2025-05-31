#pragma once
#include "Utils.hpp"

namespace core {
    struct Ray {
        Point origin;
        Vector direction;
        double wavelength; // текущая длина волны
        int depth;         // глубина рекурсии
    };

} // namespace core
