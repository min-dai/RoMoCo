#ifndef BIPED_ROS_ROS_LOAD_CONFIG_HPP
#define BIPED_ROS_ROS_LOAD_CONFIG_HPP

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <string>
#include <filesystem>
#include <cstdlib>
#include <iostream>
struct RosLoadConfig
{
    std::string config_folder;
    std::string log_path;
    RosLoadConfig(const std::string &package_name,
                  const std::string &config_folder_name,
                  const std::string &log_folder_name)
    {
        try
        {
            // Get package directory with error handling
            std::string package_folder = ament_index_cpp::get_package_share_directory(package_name);
            config_folder = package_folder + "/" + config_folder_name;

            // Validate that config folder exists
            if (!std::filesystem::exists(config_folder))
            {
                std::cerr << "Warning: Config folder does not exist: " << config_folder << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("Failed to find package '" + package_name + "': " + e.what());
        }

        std::string home = std::string(getenv("HOME"));

        // if timestamp env variable not set, use default
        std::string timestamp;
        if (getenv("LOG_FOLDER_TIMESTAMP") == NULL)
        {
            timestamp = "default";
        }
        else
        {
            timestamp = std::string(getenv("LOG_FOLDER_TIMESTAMP"));
        }

        log_path = home + "/" + log_folder_name + "/" + timestamp;
        // Create log directory if it doesn't exist
        try
        {
            std::filesystem::create_directories(log_path);
        }
        catch (const std::filesystem::filesystem_error &e)
        {
            std::cerr << "Warning: Failed to create log directory " << log_path
                      << ": " << e.what() << std::endl;
        }
    }
};

#endif // BIPED_ROS_ROS_LOAD_CONFIG_HPP