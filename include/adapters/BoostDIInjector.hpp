#pragma once
#include <boost/di.hpp>
#include "core/IConfigLoader.hpp"
#include "core/IRenderer.hpp"
#include "core/RayTracer.hpp"
#include "adapters/JsonConfigLoader.hpp"
#include "adapters/SFMLRenderer.hpp"

#define USE_JSON_CONFIG_PATH

namespace adapters {

#ifdef USE_JSON_CONFIG_PATH
    constexpr const char* DEFAULT_CONFIG_PATH = "../config.json";
#else
    constexpr const char* DEFAULT_CONFIG_PATH = "";
#endif

    inline auto createInjector() {
        namespace di = boost::di;
        return di::make_injector(
            di::bind<core::IConfigLoader>.to([&](const auto &cra) -> std::shared_ptr<core::IConfigLoader> {
                return std::make_shared<JsonConfigLoader>(DEFAULT_CONFIG_PATH);
            }),
            di::bind<core::IRenderer>().to<adapters::SFMLRenderer>(),
            di::bind<core::RayTracer>.to<core::RayTracer>()
        );
    }


} // namespace adapters
