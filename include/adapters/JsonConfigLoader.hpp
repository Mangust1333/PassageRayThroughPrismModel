#pragma once
#include <string>
#include <unordered_map>
#include <fstream>
#include "core/IConfigLoader.hpp"
#include "core/Material.hpp"
#include <nlohmann/json.hpp>

namespace adapters {

    class JsonConfigLoader : public core::IConfigLoader {
    public:
        explicit JsonConfigLoader(std::string filename);
        std::unordered_map<std::string, core::Material> loadAllMaterials() override;
        core::Material getMaterialByName(const std::string &) const override;
    private:
        std::string filename_;
    };

} // namespace adapters
