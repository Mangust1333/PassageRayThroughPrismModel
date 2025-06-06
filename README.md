# Отчёт по моделированию преломления света с дисперсией

*Автор: Кубанцев Владислав*

*Дата: 2025-05-31*

---

## Содержание

1. [Введение](#введение)
2. [Теоретическая часть](#теоретическая-часть)

   1. [Призма и законы геометрической оптики](#призма-и-законы-геометрической-оптики)
   2. [Дисперсия и зависимость показателя преломления от длины волны](#дисперсия-и-зависимость-показателя-преломления-от-длины-волны)
   3. [Рассчёт угла отклонения луча](#рассчёт-угла-отклонения-луча)
3. [Постановка задачи и функциональные требования](#постановка-задачи-и-функциональные-требования)

   1. [Общие требования](#общие-требования)
   2. [Дополнительные возможности](#дополнительные-возможности)
4. [Описание архитектуры и структуры проекта](#описание-архитектуры-и-структуры-проекта)

   1. [Используемые технологии и библиотеки](#используемые-технологии-и-библиотеки)
   2. [Архитектура (слой за слоем)](#архитектура-слой-за-слоем)
5. [Приложения](#приложения)

   1. [Полный список файлов проекта](#полный-список-файлов-проекта)

---

## Введение

Цель данного проекта — смоделировать преломление света в прозрачных призмах с учётом дисперсии (зависимость показателя преломления от длины волны). Помимо простой призмы, модель была расширена: луч может проходить через любое число фигур (многогранников), у каждой фигуры может быть произвольное число сторон, а материал для каждой фигуры выбирается из внешнего конфигурационного файла.

Отчёт описывает:

* основные физические принципы (законы геометрической и волновой оптики, дисперсию),
* постановку задачи и функциональные требования,
* архитектуру приложения (с использованием Boost.DI, CMake, SFML),
* реализацию ключевых классов: `Material`, `RayTracer`, `SFMLRenderer` и др.,
* управление симуляцией (меню переключения угла падения и материала),
* визуализацию спектра как цветной полосы.

---

## Теоретическая часть

### Призма и законы геометрической оптики

Призма — прозрачное тело (чаще всего трёхгранник), в котором под углом падает световой луч заданной длины волны λ. Основные законы, используемые при моделировании:

1. **Закон Снеллиуса (Snell’s law)**
   При переходе света из одной среды в другую с показателями преломления $n_1$ и $n_2$ угол падения $\theta_1$ и угол преломления $\theta_2$ связаны соотношением:

   $n_1 \sin \theta_1 = n_2 \sin \theta_2$

   В нашем случае $n_1 = 1$ (воздух), $n_2 = n(\lambda)$ — показатель преломления призмы, зависящий от длины волны.

3. **Преломление на двух гранях призмы**
   Луч, попадающий в призму, преломляется при входе с углами $\theta_{\text{in}}$ и $\theta_{\text{refr,in}}$. После прохождения через призму на второй грани луч выходит в воздух под углом $\theta_{\text{out}}$. Для правильного расчёта нужно учесть:

   * угол призмы (апексовый угол),
   * взаиморасположение нормалей к граням,
   * возможно полное внутреннее отражение, если $\theta$ превышает критический угол.

4. **Угол отклонения**
   Условно, для призмы с апикальным углом $A$ и падающим лучом с углом падения $i$ (внешне), угол выхода $e$ (внешний) удовлетворяет:

   $\delta(\lambda) = i + e - A,$

   где $e$ вычисляется из закона Снеллиуса. На практике в симуляции луч «рассчитывается» через две грани:

   1. попадая внутрь,
   2. внутри среды движется в “вычисленной” направленности,
   3. выходит наружу, снова преломляется.

### Дисперсия и зависимость показателя преломления от длины волны

Дисперсия — явление, при котором показатель преломления $n$ материала зависит от длины волны $\lambda$. Распространённая эмпирическая модель: **формула Кайзера (Cauchy)** или более точная **формула Селмейера (Sellmeier)**. В нашем проекте сырьём служат параметры $B_i$ и $C_i$ (обычно три пары параметров для видимого диапазона — UV и IR участки), согласно формуле:

$$
    n^2(\lambda) = 1 + \sum_{i=1}^3 \frac{B_i \, \lambda^2}{\lambda^2 - C_i},
\quad
\lambda \text{ в микрометрах}.
$$

Далее:

$$
    n(\lambda) = \sqrt{|n^2(\lambda)|}.
$$

Код в классе `Material`:

```cpp
double core::Material::refractiveIndex(double lambda_nm) const {
    if (this->special_value > 0) {
        // Специальное значение: для зеркала или искусственных “материалов”
        return this->special_value;
    }

    // Переводим λ из нанометров в микрометры
    double λ = lambda_nm / 1000.0;
    double n2 = 1.0;
    for (int i = 0; i < 3; ++i) {
        // B[i], C[i] загружены из конфигурации (Sellmeier-параметры)
        n2 += (B[i] * λ * λ) / (λ * λ - C[i]);
    }

    return std::sqrt(std::abs(n2));
}
```

### Рассчёт угла отклонения луча

Для каждой грани (отдельного отрезка полигона) мы решаем систему уравнений прямой и отрезка грани, чтобы найти точку пересечения. Если пересечение найдено (параметры $t > 0$, $u\in[0,1]$), то вычисляем:

1. нормаль к грани:

   $\mathbf{n} = \frac{(y_2 - y_1\; x_1 - x_2)}{\|\|(y_2 - y_1\; x_1 - x_2)\|\|}\quad \text{(правосторонняя нормаль)}.$
2. угол падения $\theta_1 = \arccos(\mathbf{d} \cdot \mathbf{n})$, где $\mathbf{d}$ — единичный вектор направления луча.
3. проверка **полного внутреннего отражения**:

   $\sin \theta_2 = \frac{n_1}{n_2} \sin \theta_1$

   если $|\sin \theta_2| > 1$, то отражение:

$$
     \mathbf{d_{\text{refl}}} = \mathbf{d} - 2(\mathbf{d} \cdot \mathbf{n})\mathbf{n}.
$$

4. иначе преломление:

$$
     \theta_2 = \arcsin\bigl(\tfrac{n_1}{n_2}\sin\theta_1\bigr),\quad 
     \mathbf{d_{\text{refr}}} = \text{rotate}(\mathbf{n},\mathrm{sign}(\mathbf{n} \times \mathbf{d}) \cdot \theta_2).
$$

Затем рекурсивно продолжаем трассировку «изнутри» или выходим наружу.

---

## Постановка задачи и функциональные требования

### Общие требования

1. **Основной объект моделирования**: Призма (или произвольный многоугольник) из прозрачного материала.
2. **Преломление света с учётом дисперсии**:

   * Пользователь задаёт угол падения света $\alpha$.
   * Материал призмы описывается Sellmeier-параметрами либо «специальным» значением $n=\mathrm{const}$ (например, для зеркала).
   * Для каждой длины волны $\lambda$ в диапазоне (100 нм … 2000 нм) рассчитывается $n(\lambda)$.
3. **Расчёт углов отклонения для разных длин волн**:

   * Разбить спектр на N (например, 30) равномерных значений $\lambda_i$.
   * Для каждого $\lambda_i$ проследить луч, получить конечный угол выхода $\delta(\lambda_i)$.
4. **Построение выходящего спектра**:

   * Отобразить все исходящие лучи на экране разными цветами (RGB по длине волны).
   * Визуализация цветового спектра в виде цветной полосы под графиком лучей.

### Дополнительные возможности

* **Множественные фигуры**:

  * Вместо одной призмы луч может проходить через любую последовательность многоугольников.
  * Каждая фигура задаётся количеством сторон $n_{\text{sides}}$ и радиусом вписанной окружности (или описанной) — регулярный многоугольник строится автоматически.
  * Можно добавлять/удалять фигуры произвольно (например, два-пять штук), они не пересекаются по умолчанию или пересекаются по логике.
* **Прозрачность и зеркальные поверхности**:

  * Для отдельных материалов (например, «зеркало») используется фиксированный специальный показатель $n$ (например, $\infty$ или очень большое значение), что приводит к полной внутренней отражательной способности.
* **Управление режимом**:

  * Переключение угла падения стрелками влево/вправо (мало градусов шаг).
  * Переключение материала призмы (PageUp = следующий, PageDown = предыдущий).
  * Доступные материалы загружаются из JSON-конфигурации.

---

## Описание архитектуры и структуры проекта

### Используемые технологии и библиотеки

* **C++17** (GNU C++ 13.1 / MinGW на Windows).
* **CMake** — система сборки, кроссплатформенная.
* **Boost.DI** — контейнер зависимостей (Dependency Injection).
* **nlohmann/json** — парсер JSON для загрузки данных о материалах.
* **SFML 2.4** (Simple and Fast Multimedia Library) — рендеринг графики и обработка событий.
* **STL** (библиотека стандартных шаблонов): `std::vector`, `std::array`, `std::unique_ptr`, `std::shared_ptr`, `std::unordered_map`.

### Архитектура (слой за слоем)

Проект выстроен по принципу трёх основных слоёв (минимально адаптированная многослойная архитектура):

1. **Модель (core)**

   * Описывает объекты (материалы, полигоны, лучи), методы расчёта (рефракция, трейсинг).
   * Файлы:

     * `core/Material.hpp/cpp` — класс `Material` и метод `refractiveIndex`
     * `core/Polygon.hpp/cpp` — класс `Polygon` (регулярный многоугольник)
     * `core/Ray.hpp` — структура `Ray`
     * `core/RayTracer.hpp/cpp` — основной алгоритм трассировки лучей
     * `core/Utils.hpp/cpp` — вспомогательные функции (например, `rotateVector`, `outwardNormal`, проверка полного внутреннего отражения).
   * **Интерфейсы**:

     * `core/IConfigLoader.hpp` — интерфейс загрузчика конфигурации (JSON).
     * `core/IRenderer.hpp` — интерфейс рендерера (рисование линий, фигур, очистка экрана, обработка событий).

2. **Адаптеры (adapters)**

   * **Загрузка материалов**:

     * `adapters/JsonConfigLoader.hpp/cpp` — реализация `IConfigLoader`, читает файл `config.json`.
   * **Графический рендер**:

     * `adapters/SFMLRenderer.hpp/cpp` — класс `SFMLRenderer`, реализует `IRenderer`, рисует с помощью SFML.
     * Здесь содержатся методы `drawLine`, `drawDashedLine`, `drawPolygon`, `pollEvents` инициализация окна SFML.
   * **DI-инжектор**:

     * `adapters/BoostDIInjector.hpp/cpp` — конфигурация Boost.DI: бинды для `IConfigLoader`, `IRenderer`, `RayTracer`.
     * Устанавливает, что конкретно внедряется: `JsonConfigLoader`, `SFMLRenderer`, `core::RayTracer`.

3. **Главная программа (app)**

   * `src/main.cpp` — точка входа.
   * Получает зависимости через DI, инициализирует рендерер, создает фигуры (призма и/или другие многоугольники), запускает главный цикл:

     1. Очистка экрана,
     2. Рисование контуров фигур,
     3. Трассировка лучей (`RayTracer::traceScene`),
     4. Отрисовка на экран (`renderer->display()`),
     5. Обработка событий (`renderer->pollEvents(…)`).

---

## Приложения

### Полный список файлов проекта

```
Modelirovanie_Optica/
├── CMakeLists.txt
├── config.json
├── include/
│   ├── adapters/
│   │   ├── BoostDIInjector.hpp
│   │   ├── JsonConfigLoader.hpp
│   │   └── SFMLRenderer.hpp
│   └── core/
│       ├── IConfigLoader.hpp
│       ├── IRenderer.hpp
│       ├── Material.hpp
│       ├── Material.cpp
│       ├── Polygon.hpp
│       └── Utils.hpp
├── src/
│   ├── main.cpp
│   ├── core/
│   │   ├── Polygon.cpp
│   │   ├── Ray.hpp
│   │   ├── RayTracer.hpp
│   │   ├── RayTracer.cpp
│   │   └── Utils.cpp
│   └── adapters/
│       ├── BoostDIInjector.cpp
│       ├── JsonConfigLoader.cpp
│       └── SFMLRenderer.cpp
├── include/  (если используется отдельная папка для хедеров)
└── README.md  (описание проекта)
```

* `config.json` содержит JSON-объект с параметрами материалов.
* `CMakeLists.txt` настраивает поиск SFML, Boost, nlohmann/json, формирует таргеты `RayTracingApp`.

---

**Конец отчёта**
