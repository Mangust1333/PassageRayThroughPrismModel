#pragma once
#include "core/IRenderer.hpp"
#include <SFML/Graphics.hpp>
#include <memory>
#include <core/IConfigLoader.hpp>

namespace adapters {

    class SFMLRenderer : public core::IRenderer {
    public:
        SFMLRenderer(std::shared_ptr<core::IConfigLoader> configLoader);
        ~SFMLRenderer() override;

        void initialize(unsigned int width, unsigned int height, const std::string& title) override;
        bool isOpen() override;
        void clear() override;
        void drawLine(const core::Point& a, const core::Point& b, const std::array<unsigned char,3>& rgb) override;
        void drawDashedLine(const core::Point &a, const core::Point &b, const std::array<unsigned char, 3> &rgb,
            float dashLength, float gapLength) override;
        void drawThickLine(const core::Point& a, const core::Point& b, const std::array<unsigned char,3>& rgb);
        void drawPolygon(const std::vector<core::Point>& vertices) override;
        void display() override;
        void pollEvents(double& outLightAngle, std::string& outMaterialName) override;

    private:
        std::shared_ptr<core::IConfigLoader> configLoader_;
        std::unique_ptr<sf::RenderWindow> window_;
        float scale_ = 3.0f;
        unsigned int width_ = 800;
        unsigned int height_ = 600;
        float thickness = 3.0f;
    };

} // namespace adapters
