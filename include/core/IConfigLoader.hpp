#pragma once
#include <string>
#include <unordered_map>
#include "Material.hpp"

namespace core {
    struct IConfigLoader {
        virtual ~IConfigLoader() = default;
        virtual std::unordered_map<std::string, Material> loadAllMaterials() = 0;
        [[nodiscard]] virtual Material getMaterialByName(const std::string&) const = 0;
    };

} // namespace core
