#ifndef YAML_PARSER_HPP
#define YAML_PARSER_HPP


#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>


/**
 * @brief A YAML Parser class for loading parameters of various types.
 */
class YAMLParser {
public:
    
    /**
     * @brief Constructor: Loads YAML file into memory.
     * @param file_path Path to the YAML file.
     */
    explicit YAMLParser(const std::string& file_path);

    /**
     * @brief Default constructor: Initializes an empty YAML node.
     */
    YAMLParser();

    /**
     * @brief Loads the YAML file into memory.
     * @param file_path Path to the YAML file.
     */
    void Init(const std::string& file_path);

    /**
     * @brief Retrieves an Eigen::VectorXd from YAML.
     * @param key_path Slash-separated key path (e.g., "qp/OutputKP").
     * @return Eigen::VectorXd containing the values.
     */
    Eigen::VectorXd get_VectorXd(const std::string& key_path) const;

    /**
     * @brief Retrieves an integer from YAML.
     * @param key_path Slash-separated key path.
     * @return Integer value.
     */
    int get_int(const std::string& key_path) const;

    /**
     * @brief Retrieves a double from YAML.
     * @param key_path Slash-separated key path.
     * @return Double value.
     */
    double get_double(const std::string& key_path) const;

    /**
     * @brief Retrieves a string from YAML.
     * @param key_path Slash-separated key path.
     * @return String value.
     */
    std::string get_string(const std::string &key_path) const;

    /**
     * @brief Retrieves a vector of strings from YAML.
     * @param key_path Slash-separated key path.
     * @return std::vector<std::string> containing the values.
     */
    std::vector<std::string> get_string_vector(const std::string &key_path) const;

    /**
     * @brief Retrieves a boolean from YAML.
     * @param key_path Slash-separated key path.
     * @return Boolean value.
     */
    bool get_bool(const std::string& key_path) const;



private:
    YAML::Node root; ///< Stores the loaded YAML file.

    /**
     * @brief Traverses the YAML tree based on a slash-separated key path.
     * @param key_path Slash-separated key path (e.g., "qp/OutputKP").
     * @return YAML::Node corresponding to the key.
     */
    YAML::Node getNode(const std::string& key_path) const;

    bool initialized = false;

};

#endif // YAML_PARSER_HPP