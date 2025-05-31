#include <iostream>
#include <memory>
#include "adapters/BoostDIInjector.hpp"
#include "core/IRenderer.hpp"
#include "core/IConfigLoader.hpp"
#include "core/RayTracer.hpp"
#include "core/Polygon.hpp"
#include "core/Utils.hpp"

int main() {
    // 1) Создаём DI-инжектор
    auto injector = adapters::createInjector();

    // 2) Получаем экземпляры через DI
    auto configLoader = injector.create<std::shared_ptr<core::IConfigLoader>>();
    const auto renderer = injector.create<std::shared_ptr<core::IRenderer>>();
    const auto rayTracer = injector.create<std::shared_ptr<core::RayTracer>>();


    // 3) Инициализируем окно
    int windowWidth = 1000;
    int windowHeight = 1000;
    renderer->initialize(windowWidth, windowHeight, "Множественные преломления");

    // 4) Подготавливаем фигуры (шестиугольник и пятиугольник)
    using namespace core;
    std::vector<Polygon> figures;
    Polygon hexagon;
    hexagon.vertices = getRegularPolygonCorners(6, 30.0, { -40.0, 0.0 });
    figures.push_back(hexagon);
    Polygon pentagon;
    pentagon.vertices = getRegularPolygonCorners(30, 25.0, { 40.0, 0.0 });
    figures.push_back(pentagon);

    // 5) По умолчанию материал = «diamond», угол = 0
    std::string currentMaterial = "diamond";
    double currentAngle = 0.0;

    // 6) Главный цикл
    while (renderer->isOpen()) {
        renderer->clear();

        // 7) Рисуем контуры фигур
        for (auto& poly : figures) {
            renderer->drawPolygon(poly.vertices);
        }

        // 8) Трассируем лучи (внутри RayTracer происходят отрисовки лучей)
        rayTracer->traceScene(figures, currentMaterial, currentAngle);

        // 9) Выводим все
        renderer->display();

        // 10) Обрабатываем события (можем менять currentAngle и currentMaterial)
        renderer->pollEvents(currentAngle, currentMaterial);
    }

    return 0;
}
