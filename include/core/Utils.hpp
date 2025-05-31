#pragma once
#include <cmath>
#include <utility>
#include <vector>
#include <array>

namespace core {

    using Point = std::array<double, 2>;
    using Vector = std::array<double, 2>;

    // Поворот вектора на угол angle (в радианах)
    inline Vector rotateVector(const Vector& v, double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        return { v[0]*c - v[1]*s, v[0]*s + v[1]*c };
    }

    // Внешняя нормаль к ребру (edge = end - start)
    inline Vector outwardNormal(const Vector& edge) {
        Vector n = { edge[1], -edge[0] };
        double len = std::hypot(n[0], n[1]);
        return { n[0]/len, n[1]/len };
    }

    // Проверка, происходит ли полное внутреннее отражение
    inline bool isTotalInternalReflection(double n_in, double n_out, double thetaIncident) {
        if (n_out == std::numeric_limits<double>::infinity()) {
            return true;
        }
        double sin_theta_trans = (n_in / n_out) * std::sin(thetaIncident);
        return std::abs(sin_theta_trans) > 1.0;
    }

    // Генерация вершин правильного многоугольника
    inline std::vector<Point> getRegularPolygonCorners(int nSides, double radius, const Point& center) {
        std::vector<Point> vertices;
        vertices.reserve(nSides);
        for (int i = 0; i < nSides; ++i) {
            double angle = 2.0 * M_PI * i / nSides;
            double x = radius * std::sin(angle) + center[0];
            double y = radius * std::cos(angle) + center[1];
            vertices.push_back({x, y});
        }
        return vertices;
    }
} // namespace core
