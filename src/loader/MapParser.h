#ifndef MAP_PARSER_H
#define MAP_PARSER_H

#include <string>
#include <nlohmann/json.hpp>

namespace MapParser {
    bool getInt(const nlohmann::json& obj, const char* key, int& value, std::string& error);
    bool getDouble(const nlohmann::json& obj, const char* key, double& value, std::string& error);
    bool getIntOptional(const nlohmann::json& obj, const char* key, int& value, std::string& error);
    bool getBoolOptional(const nlohmann::json& obj, const char* key, bool& value, std::string& error);
    bool getDoubleOptional(const nlohmann::json& obj, const char* key, double& value, std::string& error);
    bool getStringOptional(const nlohmann::json& obj, const char* key, std::string& value, std::string& error);
}

#endif // MAP_PARSER_H
