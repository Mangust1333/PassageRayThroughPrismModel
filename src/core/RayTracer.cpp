#include <core/RayTracer.hpp>

core::RayTracer::RayTracer(std::shared_ptr<IConfigLoader> cfgLoader, std::shared_ptr<IRenderer> renderer):
configLoader_(std::move(cfgLoader)), renderer_(renderer.get()) {
    materials_ = configLoader_->loadAllMaterials();
}

void core::RayTracer::traceScene(const std::vector<Polygon> &figures, const std::string &materialName, double lightAngleDeg) {
    auto matIt = materials_.find(materialName);
    if (matIt == materials_.end()) {
        throw std::runtime_error("Material not found: " + materialName);
    }
    const Material& material = matIt->second;

    double angleRad = lightAngleDeg * M_PI / 180.0;
    Vector initDir = { std::cos(angleRad), std::sin(angleRad) };
    Point initOrigin = { -100.0, 20.0 };

    constexpr double downLight = 100.0;
    constexpr double upLight = 2000.0;
    const int N = 30;
    for (int i = 0; i < N; ++i) {
        double lambda = downLight + (upLight - downLight) * i / (N - 1);
        Ray ray{ initOrigin, initDir, lambda, 0 };
        traceRayRecursive(ray, figures, material);
    }
}

void core::RayTracer::traceRayRecursive(const Ray &ray, const std::vector<Polygon> &figures, const Material &material) {
    if (ray.depth > 10) return;
    double closestT = std::numeric_limits<double>::infinity();
    Point intersection{};
    const Polygon* hitPoly = nullptr;
    Vector hitEdge{};

    for (auto& poly : figures) {
        const auto& verts = poly.vertices;
        int m = static_cast<int>(verts.size());
        for (int i = 0; i < m; ++i) {
            Point A = verts[i];
            Point B = verts[(i+1)%m];
            Vector edge = { B[0] - A[0], B[1] - A[1] };

            double a11 = ray.direction[0];
            double a12 = -edge[0];
            double a21 = ray.direction[1];
            double a22 = -edge[1];
            double b1 = A[0] - ray.origin[0];
            double b2 = A[1] - ray.origin[1];
            double det = a11*a22 - a12*a21;
            if (std::fabs(det) < 1e-8) continue;
            double t = ( b1 * a22 - b2 * a12 ) / det;
            double u = (-b1 * a21 + b2 * a11 ) / det;
            if (t > 1e-6 && u >= 0.0 && u <= 1.0 && t < closestT) {
                closestT = t;
                intersection = { ray.origin[0] + t*ray.direction[0],
                    ray.origin[1] + t*ray.direction[1] };
                hitPoly = &poly;
                hitEdge = edge;
            }
        }
    }

    if (!hitPoly) {
        Point farPoint = { ray.origin[0] + ray.direction[0]*1000.0,
            ray.origin[1] + ray.direction[1]*1000.0 };
        auto rgb = wavelengthToRGB(ray.wavelength);
        renderer_->drawLine(ray.origin, farPoint, rgb);
        return;
    }

    auto rgb = wavelengthToRGB(ray.wavelength);
    renderer_->drawLine(ray.origin, intersection, rgb);

    Vector normal = outwardNormal(hitEdge);

    double n_in = 1.0;
    double n_out = material.refractiveIndex(ray.wavelength);

    double cosTheta1 = ray.direction[0]*normal[0] + ray.direction[1]*normal[1];
    double theta1 = std::acos(std::clamp(cosTheta1, -1.0, 1.0));

    if (isTotalInternalReflection(n_in, n_out, theta1)) {
        Vector reflDir = {
            ray.direction[0] - 2.0 * (ray.direction[0]*normal[0] + ray.direction[1]*normal[1]) * normal[0],
            ray.direction[1] - 2.0 * (ray.direction[0]*normal[0] + ray.direction[1]*normal[1]) * normal[1]
        };
        Ray reflRay{ intersection, reflDir, ray.wavelength, ray.depth + 1 };
        traceRayRecursive(reflRay, figures, material);
    } else {
        // Преломление внутрь
        double sinTheta2 = (n_in / n_out) * std::sin(theta1);
        double theta2 = std::asin(std::clamp(sinTheta2, -1.0, 1.0));
        // Определяем знак поворота (cross product в 2D)
        double crossZ = normal[0]*ray.direction[1] - normal[1]*ray.direction[0];
        double sign = (crossZ > 0.0) ? 1.0 : -1.0;
        Vector dirIn = rotateVector(normal, sign * theta2);
        Ray insideRay{ intersection, dirIn, ray.wavelength, ray.depth + 1 };
        traceRayInside(*hitPoly, insideRay, figures, material);
    }
}

void core::RayTracer::traceRayInside(const Polygon &poly, const Ray &ray, const std::vector<Polygon> &allFigures,
    const Material &material) {
    Ray currentRay = ray;
    for (int bounce = 0; bounce < 10; ++bounce) {
        double closestT = std::numeric_limits<double>::infinity();
        Point intersection{};
        Vector hitEdge{};
        for (int i = 0; i < static_cast<int>(poly.vertices.size()); ++i) {
            Point A = poly.vertices[i];
            Point B = poly.vertices[(i+1) % poly.vertices.size()];
            Vector edge = { B[0] - A[0], B[1] - A[1] };

            double a11 = currentRay.direction[0];
            double a12 = -edge[0];
            double a21 = currentRay.direction[1];
            double a22 = -edge[1];
            double b1 = A[0] - currentRay.origin[0];
            double b2 = A[1] - currentRay.origin[1];
            double det = a11*a22 - a12*a21;
            if (std::fabs(det) < 1e-8) continue;
            double t = ( b1 * a22 - b2 * a12 ) / det;
            double u = (-b1 * a21 + b2 * a11 ) / det;
            if (t > 1e-6 && u >= 0.0 && u <= 1.0 && t < closestT) {
                closestT = t;
                intersection = { currentRay.origin[0] + t*currentRay.direction[0],
                    currentRay.origin[1] + t*currentRay.direction[1] };
                hitEdge = edge;
            }
        }

        if (closestT == std::numeric_limits<double>::infinity()) {
            return;
        }

        auto rgb = wavelengthToRGB(currentRay.wavelength);
        renderer_->drawDashedLine(currentRay.origin, intersection, rgb, 5.0f, 3.0f); // можно

        // Нормаль на выходе (обратная)
        Vector normalOut = outwardNormal(hitEdge);
        double cosTheta = -(currentRay.direction[0]*normalOut[0] + currentRay.direction[1]*normalOut[1]);
        double theta = std::acos(std::clamp(cosTheta, -1.0, 1.0));

        double n_in = material.refractiveIndex(currentRay.wavelength);
        double n_out = 1.0;

        if (isTotalInternalReflection(n_in, n_out, theta)) {
            // внутреннее отражение внутри
            Vector reflDir = {
                currentRay.direction[0] - 2.0 * (currentRay.direction[0]*normalOut[0] + currentRay.direction[1]*normalOut[1]) * normalOut[0],
                currentRay.direction[1] - 2.0 * (currentRay.direction[0]*normalOut[0] + currentRay.direction[1]*normalOut[1]) * normalOut[1]
            };
            currentRay.origin = intersection;
            currentRay.direction = reflDir;
            currentRay.depth += 1;
            continue;
        } else {
            // выходим из фигуры: преломляем луч в окружающую среду
            double sinThetaOut = (n_in / n_out) * std::sin(theta);
            double thetaOut = std::asin(std::clamp(sinThetaOut, -1.0, 1.0));
            double crossZ = (-normalOut[0])*currentRay.direction[1] - (-normalOut[1])*currentRay.direction[0];
            double sign = (crossZ > 0.0) ? 1.0 : -1.0;
            Vector dirOut = rotateVector({-normalOut[0], -normalOut[1]}, sign * thetaOut);
            Ray outRay{ intersection, dirOut, currentRay.wavelength, currentRay.depth + 1 };
            traceRayRecursive(outRay, allFigures, material);
            return;
        }
    }
}

std::array<unsigned char, 3> core::RayTracer::wavelengthToRGB(double wavelength) {
    // Аналог hsv_to_rgb из Python, но здесь вернём просто RGB в байтах
    double hue = (2000.0 - wavelength) / (2000.0 - 100.0) * 0.75;
    // Простейшее преобразование HSV->RGB:
    double h = hue * 6.0;
    int sector = static_cast<int>(std::floor(h));
    double f = h - sector;
    double p = 0.0;
    double q = 1.0 - f;
    double t = f;
    double r, g, b;
    switch (sector % 6) {
        case 0: r=1; g=t; b=0; break;
        case 1: r=q; g=1; b=0; break;
        case 2: r=0; g=1; b=t; break;
        case 3: r=0; g=q; b=1; break;
        case 4: r=t; g=0; b=1; break;
        case 5: r=1; g=0; b=q; break;
    }
    // Возвращаем цвет в диапазоне 0–255
    return {
        static_cast<unsigned char>(std::round(r*255)),
        static_cast<unsigned char>(std::round(g*255)),
        static_cast<unsigned char>(std::round(b*255))
    };
}
