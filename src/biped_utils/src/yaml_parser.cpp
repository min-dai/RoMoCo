#include "biped_utils/yaml_parser.hpp"
#include <sstream>
#include <stdexcept>
#include <string>

/**
 * @brief Constructor: Loads the YAML file into memory.
 */
YAMLParser::YAMLParser(const std::string &file_path)
{
    root = YAML::LoadFile(file_path);
    initialized = true;
}

/**
 * @brief Default constructor: Initializes an empty YAML node.
 */
YAMLParser::YAMLParser()
{
    root = YAML::Node();
    initialized = false;
}

/**
 * @brief Loads the YAML file into memory.
 */
void YAMLParser::Init(const std::string &file_path)
{
    root = YAML::LoadFile(file_path);
    initialized = true;
}

/**
 * @brief Traverses the YAML tree based on a slash-separated key path.
 */
YAML::Node YAMLParser::getNode(const std::string &key_path) const
{

    // make sure is initialized'
    if (!initialized)
    {
        throw std::runtime_error("YAMLParser not initialized");
    }

    std::istringstream ss(key_path);
    std::string key;
    YAML::Node node = YAML::Clone(root);

    // Split the path by '/'
    while (std::getline(ss, key, '/'))
    {
        if (!node.IsMap())
        {
            throw std::runtime_error("Invalid key path: " + key_path + " (expected a mapping but found a list)");
        }
        if (!node[key])
        {
            throw std::runtime_error("Missing key: " + key_path);
        }
        node = node[key];
    }
    return node; // Return the final node
}

bool YAMLParser::getNodeOptional(const std::string& key_path, YAML::Node& node) const
{
    // Make sure is initialized
    if (!initialized)
    {
        throw std::runtime_error("YAMLParser not initialized");
    }

    node = YAML::Clone(root);
    std::istringstream ss(key_path);
    std::string key;

    // Split the path by '/'
    while (std::getline(ss, key, '/'))
    {
        if (!node.IsMap())
        {
            return false; // Not a mapping, cannot proceed
        }
        if (!node[key])
        {
            return false; // Key does not exist
        }
        node = node[key];
    }
    return true; // Successfully retrieved the node
}

/**
 * @brief Retrieves an Eigen::VectorXd from YAML.
 */
Eigen::VectorXd YAMLParser::get_VectorXd(const std::string &key_path) const
{
    YAML::Node node = getNode(key_path);

    if (!node.IsSequence())
    {
        throw std::runtime_error("Expected a sequence at key path: " + key_path);
    }

    int size = node.size();
    Eigen::VectorXd vec(size);
    for (int i = 0; i < size; i++)
    {
        vec[i] = node[i].as<double>();
    }

    return vec;
}

bool YAMLParser::get_VectorXd_optional(const std::string &key_path, Eigen::VectorXd &result) const
{
    YAML::Node node;
    bool exists = getNodeOptional(key_path, node);

    if (!exists)
    {
        return false; // Key does not exist
    }
    else
    {
        if (!node.IsSequence())
        {
            throw std::runtime_error("Expected a sequence at key path: " + key_path);
        }
        int size = node.size();
        result.resize(size);
        for (int i = 0; i < size; i++)
        {
            result[i] = node[i].as<double>();
        }
    }
    return true; // Successfully retrieved the value
}

/**
 * @brief Retrieves an integer from YAML.
 */
int YAMLParser::get_int(const std::string &key_path) const
{
    YAML::Node node = getNode(key_path);
    int result = node.as<int>();
    return result;
}

/**
 * @brief Retrieves a double from YAML.
 */
double YAMLParser::get_double(const std::string &key_path) const
{
    YAML::Node node = getNode(key_path);
    return node.as<double>();
}

/**
 * @brief Retrieves a string from YAML.
 */
std::string YAMLParser::get_string(const std::string &key_path) const
{
    YAML::Node node = getNode(key_path);
    return node.as<std::string>();
}

std::vector<std::string> YAMLParser::get_string_vector(const std::string &key_path) const
{
    YAML::Node node = getNode(key_path);

    if (!node)
    {
        throw std::runtime_error("Invalid key or not a sequence: " + key_path);
    }

    std::vector<std::string> values;
    for (const auto &item : node)
    {
        values.push_back(item.as<std::string>());
    }

    return values;
}

bool YAMLParser::get_string_vector_optional(const std::string &key_path, std::vector<std::string> &values) const
{
    YAML::Node node;
    bool exists = getNodeOptional(key_path, node);

    if (!exists)
    {
        return false; // Key does not exist
    }
    else
    {
        if (!node.IsSequence())
        {
            throw std::runtime_error("Expected a sequence at key path: " + key_path);
        }
        values.clear();
        for (const auto &item : node)
        {
            values.push_back(item.as<std::string>());
        }
    }
    return true; // Successfully retrieved the value
}

/**
 * @brief Retrieves a boolean from YAML.
 */
bool YAMLParser::get_bool(const std::string &key_path) const
{
    YAML::Node node = getNode(key_path);
    return node.as<bool>();
}