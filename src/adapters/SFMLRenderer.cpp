#include "adapters/SFMLRenderer.hpp"
#include <iostream>

namespace adapters {

SFMLRenderer::SFMLRenderer(std::shared_ptr<core::IConfigLoader> configLoader)
    : window_(nullptr), configLoader_(std::move(configLoader))
{}

SFMLRenderer::~SFMLRenderer() = default;

void SFMLRenderer::initialize(unsigned int width, unsigned int height, const std::string& title) {
    window_ = std::make_unique<sf::RenderWindow>(sf::VideoMode({width, height}, 32), title);
    window_->setFramerateLimit(60);
    width_ = width;
    height_ = height;
}

bool SFMLRenderer::isOpen() {
    return window_ && window_->isOpen();
}

void SFMLRenderer::clear() {
    window_->clear(sf::Color::White);
}

void SFMLRenderer::drawLine(const core::Point& a, const core::Point& b, const std::array<unsigned char,3>& rgb) {
    drawThickLine(a, b, rgb);
}

void SFMLRenderer::drawDashedLine(const core::Point& a, const core::Point& b, const std::array<unsigned char,3>& rgb, float dashLength, float gapLength) {
    sf::Vector2f pointA(a[0] * scale_ + width_ / 2, height_ / 2 - a[1] * scale_);
    sf::Vector2f pointB(b[0] * scale_ + width_ / 2, height_ / 2 - b[1] * scale_);

    sf::Vector2f direction = pointB - pointA;
    float totalLength = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    sf::Vector2f unitDir = direction / totalLength;

    float drawnLength = 0.f;
    bool drawSegment = true;
    while (drawnLength < totalLength) {
        float segmentLength = drawSegment ? dashLength : gapLength;
        if (drawnLength + segmentLength > totalLength) {
            segmentLength = totalLength - drawnLength;
        }

        if (drawSegment) {
            sf::Vector2f segmentStart = pointA + unitDir * drawnLength;
            sf::Vector2f segmentEnd = segmentStart + unitDir * segmentLength;
            sf::Vertex line[] = {
                {segmentStart, sf::Color(rgb[0], rgb[1], rgb[2])},
                {segmentEnd, sf::Color(rgb[0], rgb[1], rgb[2])}
            };
            window_->draw(line, 2, sf::PrimitiveType::Lines);
        }
        drawnLength += segmentLength;
        drawSegment = !drawSegment;
    }
}


void SFMLRenderer::drawThickLine(const core::Point& a, const core::Point& b, const std::array<unsigned char,3>& rgb) {
    sf::Vector2f pointA(a[0] * scale_ + width_ / 2, height_ / 2 - a[1] * scale_);
    sf::Vector2f pointB(b[0] * scale_ + width_ / 2, height_ / 2 - b[1] * scale_);

    sf::Vector2f direction = pointB - pointA;
    float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    sf::Vector2f unitDir = direction / length;
    sf::Vector2f unitPerp(-unitDir.y, unitDir.x);  // перпендикулярный вектор

    sf::RectangleShape rectangle(sf::Vector2f(length, thickness));
    rectangle.setPosition(pointA);
    rectangle.setFillColor(sf::Color(rgb[0], rgb[1], rgb[2]));
    rectangle.setRotation(sf::Angle(sf::degrees(std::atan2(direction.y, direction.x) * 180.f / 3.14159265f)));
    window_->draw(rectangle);
}


void SFMLRenderer::drawPolygon(const std::vector<core::Point>& vertices) {
    sf::ConvexShape polyShape;
    polyShape.setPointCount(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        polyShape.setPoint(i, sf::Vector2f(
            static_cast<float>(vertices[i][0] * scale_ + width_ / 2),
            static_cast<float>(height_ / 2 - vertices[i][1] * scale_)
            ));
    }
    polyShape.setOutlineColor(sf::Color::Black);
    polyShape.setFillColor(sf::Color::Transparent);
    polyShape.setOutlineThickness(1.0f);
    window_->draw(polyShape);
}

void SFMLRenderer::display() {
    window_->display();
}

void SFMLRenderer::pollEvents(double& outLightAngle, std::string& outMaterialName) {
    static std::vector<std::string> materials;

    if (materials.empty() && configLoader_) {
        auto matsMap = configLoader_->loadAllMaterials();
        for (const auto& [name, mat] : matsMap) {
            materials.push_back(name);
        }
        if (!materials.empty()) {
            outMaterialName = materials[0];
        }
    }

    static int currentMaterialIndex = 0;

    while (auto event = window_->pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window_->close();
        }
        if (event->is<sf::Event::KeyPressed>()) {
            auto key = event->getIf<sf::Event::KeyPressed>()->code;

            if (key == sf::Keyboard::Key::Escape) {
                window_->close();
            }
            else if (key == sf::Keyboard::Key::Left) {
                outLightAngle -= 1.0;
                if (outLightAngle < 0.0) outLightAngle += 360.0;
            }
            else if (key == sf::Keyboard::Key::Right) {
                outLightAngle += 1.0;
                if (outLightAngle >= 360.0) outLightAngle -= 360.0;
            }
            else if (key == sf::Keyboard::Key::PageUp) {
                if (!materials.empty()) {
                    currentMaterialIndex = (currentMaterialIndex + 1) % materials.size();
                    outMaterialName = materials[currentMaterialIndex];
                }
            }
            else if (key == sf::Keyboard::Key::PageDown) {
                if (!materials.empty()) {
                    currentMaterialIndex = (currentMaterialIndex - 1 + materials.size()) % materials.size();
                    outMaterialName = materials[currentMaterialIndex];
                }
            }
        }
    }
}


} // namespace adapters
