#pragma once
#include <vector>
#include <memory>
#include <stdexcept>
#include <bits/stl_algo.h>

#include "IRenderer.hpp"
#include "IConfigLoader.hpp"
#include "Utils.hpp"
#include "Polygon.hpp"
#include "Material.hpp"
#include "Ray.hpp"

namespace core {

class RayTracer {
public:
    RayTracer(std::shared_ptr<IConfigLoader> cfgLoader, std::shared_ptr<IRenderer> renderer);

    void traceScene(const std::vector<Polygon>& figures,
                    const std::string& materialName,
                    double lightAngleDeg);

private:
    std::shared_ptr<IConfigLoader> configLoader_;
    std::unique_ptr<IRenderer> renderer_;
    std::unordered_map<std::string, Material> materials_;

    void traceRayRecursive(const Ray& ray,
                           const std::vector<Polygon>& figures,
                           const Material& material);

    void traceRayInside(const Polygon& poly,
                        const Ray& ray,
                        const std::vector<Polygon>& allFigures,
                        const Material& material);

    static std::array<unsigned char,3> wavelengthToRGB(double wavelength);
};

} // namespace core
