#include "MapParser.h"

using nlohmann::json;

namespace MapParser {

bool getInt(const json& obj, const char* key, int& value, std::string& error) {
    if (!obj.contains(key)) {
        error = std::string("Missing field: ") + key;
        return false;
    }
    if (!obj.at(key).is_number_integer()) {
        error = std::string("Field must be integer: ") + key;
        return false;
    }
    value = obj.at(key).get<int>();
    return true;
}

bool getDouble(const json& obj, const char* key, double& value, std::string& error) {
    if (!obj.contains(key)) {
        error = std::string("Missing field: ") + key;
        return false;
    }
    if (!obj.at(key).is_number()) {
        error = std::string("Field must be numeric: ") + key;
        return false;
    }
    value = obj.at(key).get<double>();
    return true;
}

bool getIntOptional(const json& obj, const char* key, int& value, std::string& error) {
    if (!obj.contains(key)) {
        return true;
    }
    if (!obj.at(key).is_number_integer()) {
        error = std::string("Field must be integer: ") + key;
        return false;
    }
    value = obj.at(key).get<int>();
    return true;
}

bool getBoolOptional(const json& obj, const char* key, bool& value, std::string& error) {
    if (!obj.contains(key)) {
        return true;
    }
    if (!obj.at(key).is_boolean()) {
        error = std::string("Field must be boolean: ") + key;
        return false;
    }
    value = obj.at(key).get<bool>();
    return true;
}

bool getDoubleOptional(const json& obj, const char* key, double& value, std::string& error) {
    if (!obj.contains(key)) {
        return true;
    }
    if (!obj.at(key).is_number()) {
        error = std::string("Field must be numeric: ") + key;
        return false;
    }
    value = obj.at(key).get<double>();
    return true;
}

bool getStringOptional(const json& obj, const char* key, std::string& value, std::string& error) {
    if (!obj.contains(key)) {
        return true;
    }
    if (!obj.at(key).is_string()) {
        error = std::string("Field must be string: ") + key;
        return false;
    }
    value = obj.at(key).get<std::string>();
    return true;
}

} // namespace MapParser
