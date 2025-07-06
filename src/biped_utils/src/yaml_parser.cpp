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

/**
 * @brief Retrieves a boolean from YAML.
 */
bool YAMLParser::get_bool(const std::string &key_path) const
{
    YAML::Node node = getNode(key_path);
    return node.as<bool>();
}