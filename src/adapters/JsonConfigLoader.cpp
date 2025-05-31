#include <utility>

#include "adapters/JsonConfigLoader.hpp"
using json = nlohmann::json;

namespace adapters {

    JsonConfigLoader::JsonConfigLoader(std::string filename) : filename_(std::move(filename)) {}

    std::unordered_map<std::string, core::Material> JsonConfigLoader::loadAllMaterials() {
        std::ifstream inFile(filename_);
        if (!inFile.is_open()) {
            throw std::runtime_error("Не удалось открыть файл: " + filename_);
        }
        json j;
        inFile >> j;

        std::unordered_map<std::string, core::Material> result;
        for (auto& [matName, matObj] : j.items()) {
            core::Material m;
            if (matName == "mirror") {
                m.special_value = std::numeric_limits<double>::infinity();
                result[matName] = m;
                continue;
            }
            m.special_value = 0.0;
            for (int i = 0; i < 3; ++i) {
                m.B[i] = matObj["B"][i].get<double>();
                m.C[i] = matObj["C"][i].get<double>();
            }
            result[matName] = m;
        }
        return result;
    }

    core::Material JsonConfigLoader::getMaterialByName(const std::string& name) const {
        std::ifstream inFile(filename_);
        if (!inFile.is_open()) {
            throw std::runtime_error("Не удалось открыть файл: " + filename_);
        }

        json j;
        inFile >> j;

        auto it = j.find(name);
        if (it == j.end()) {
            throw std::runtime_error("Материал не найден: " + name);
        }

        const auto& matObj = *it;
        core::Material m;

        if (name == "mirror") {
            m.special_value = std::numeric_limits<double>::infinity();
            return m;
        }

        m.special_value = 0.0;
        for (int i = 0; i < 3; ++i) {
            m.B[i] = matObj["B"][i].get<double>();
            m.C[i] = matObj["C"][i].get<double>();
        }

        return m;
    }
} // namespace adapters
