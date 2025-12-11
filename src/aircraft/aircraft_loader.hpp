#pragma once

#include "aircraft.hpp"
#include "../aerodynamics/aero_data.hpp"
#include <string>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <filesystem>
#include <memory>

// Simple JSON parser for aircraft configuration
// Expects format: { "key": value, ... }
class AircraftLoader
{
public:
    static Aircraft loadFromJSON(const std::string &filepath)
    {
        // Try to open the file with the given path
        std::ifstream file(filepath);

        // If that fails, try relative to various common locations
        if (!file.is_open())
        {
            // Build a helpful error message with absolute path
            std::filesystem::path absPath;
            try
            {
                absPath = std::filesystem::absolute(filepath);
            }
            catch (...)
            {
                absPath = filepath;
            }

            throw std::runtime_error("Failed to open aircraft config file: " + filepath +
                                     "\n  Absolute path tried: " + absPath.string());
        }

        std::string content((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
        file.close();

        // Parse JSON manually (simple approach for our known structure)
        Aircraft ac;
        ac.mass = parseDouble(content, "mass");
        ac.S = parseDouble(content, "S");
        ac.CL_alpha = parseDouble(content, "CL_alpha");
        ac.CD0 = parseDouble(content, "CD0");
        ac.k = parseDouble(content, "k");
        ac.maxThrust = parseDouble(content, "maxThrust");

        // Check for optional aeroDataFile field
        std::string aeroFile = parseString(content, "aeroDataFile");
        if (!aeroFile.empty())
        {
            ac.aeroDataFile = aeroFile;

            // Try to load the aero data file
            std::filesystem::path configDir = std::filesystem::path(filepath).parent_path();
            std::filesystem::path aeroPath = configDir / aeroFile;

            try
            {
                ac.aeroTable = std::make_shared<AeroDataTable>(AeroDataTable::loadFromCSV(aeroPath.string()));
            }
            catch (const std::exception &e)
            {
                // If loading fails, fall back to legacy parameters
                // (Could also throw here if you want to enforce aero data)
                ac.aeroTable = nullptr;
            }
        }

        return ac;
    }

private:
    static double parseDouble(const std::string &json, const std::string &key)
    {
        // Find the key in the JSON string
        std::string searchKey = "\"" + key + "\"";
        size_t keyPos = json.find(searchKey);
        if (keyPos == std::string::npos)
        {
            throw std::runtime_error("Key not found in JSON: " + key);
        }

        // Find the colon after the key
        size_t colonPos = json.find(':', keyPos);
        if (colonPos == std::string::npos)
        {
            throw std::runtime_error("Invalid JSON format for key: " + key);
        }

        // Extract the value (skip whitespace and stop at comma or closing brace)
        size_t valueStart = colonPos + 1;
        while (valueStart < json.length() && (json[valueStart] == ' ' || json[valueStart] == '\t' || json[valueStart] == '\n'))
        {
            valueStart++;
        }

        size_t valueEnd = valueStart;
        while (valueEnd < json.length() && json[valueEnd] != ',' && json[valueEnd] != '}' && json[valueEnd] != '\n')
        {
            valueEnd++;
        }

        std::string valueStr = json.substr(valueStart, valueEnd - valueStart);

        // Remove trailing whitespace
        while (!valueStr.empty() && (valueStr.back() == ' ' || valueStr.back() == '\t' || valueStr.back() == '\r'))
        {
            valueStr.pop_back();
        }

        try
        {
            return std::stod(valueStr);
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("Failed to parse value for key '" + key + "': " + valueStr);
        }
    }

    static std::string parseString(const std::string &json, const std::string &key)
    {
        // Find the key in the JSON string
        std::string searchKey = "\"" + key + "\"";
        size_t keyPos = json.find(searchKey);
        if (keyPos == std::string::npos)
        {
            return ""; // Key not found, return empty string
        }

        // Find the colon after the key
        size_t colonPos = json.find(':', keyPos);
        if (colonPos == std::string::npos)
        {
            return "";
        }

        // Skip whitespace after colon
        size_t valueStart = colonPos + 1;
        while (valueStart < json.length() && (json[valueStart] == ' ' || json[valueStart] == '\t' || json[valueStart] == '\n'))
        {
            valueStart++;
        }

        // Check if value is a string (starts with quote)
        if (valueStart >= json.length() || json[valueStart] != '"')
        {
            return "";
        }

        // Find closing quote
        size_t valueEnd = valueStart + 1;
        while (valueEnd < json.length() && json[valueEnd] != '"')
        {
            valueEnd++;
        }

        if (valueEnd >= json.length())
        {
            return "";
        }

        return json.substr(valueStart + 1, valueEnd - valueStart - 1);
    }
};
