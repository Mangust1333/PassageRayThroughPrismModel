#pragma once
#include "Utils.hpp"
#include <string>
#include <vector>

namespace core {

    struct IRenderer {
        virtual ~IRenderer() = default;
        virtual void initialize(unsigned int width, unsigned int height, const std::string& title) = 0;
        virtual bool isOpen() = 0;
        virtual void clear() = 0;
        virtual void drawLine(const Point& a, const Point& b, const std::array<unsigned char,3>& rgb) = 0;
        virtual void drawDashedLine(
            const core::Point& a,
            const core::Point& b,
            const std::array<unsigned char,3>& rgb,
            float dashLength,
            float gapLength
            ) = 0;
        virtual void drawPolygon(const std::vector<Point>& vertices) = 0;
        virtual void display() = 0;
        virtual void pollEvents(double& outLightAngle, std::string& outMaterialName) = 0;
    };
} // namespace core
