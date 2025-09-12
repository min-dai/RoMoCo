#ifndef BIPED_ROS_ROS_LOAD_CONFIG_HPP
#define BIPED_ROS_ROS_LOAD_CONFIG_HPP

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <string>
#include <filesystem>
#include <cstdlib>
#include <iostream>

namespace romoco
{
    // Utility function for recursive copy
    inline void CopyFilesRecursively(const std::filesystem::path &source_dir,
                                     const std::filesystem::path &target_dir)
    {
        if (!std::filesystem::exists(source_dir) ||
            !std::filesystem::is_directory(source_dir))
        {
            std::cerr << "Warning: Source directory does not exist or is not a directory: "
                      << source_dir << std::endl;
            return;
        }

        for (const auto &entry : std::filesystem::directory_iterator(source_dir))
        {
            const auto &path = entry.path();
            auto target_path = target_dir / path.filename();

            try
            {
                if (std::filesystem::is_regular_file(path))
                {
                    std::filesystem::copy_file(
                        path, target_path, std::filesystem::copy_options::overwrite_existing);
                }
                else if (std::filesystem::is_directory(path))
                {
                    std::filesystem::create_directories(target_path);
                    CopyFilesRecursively(path, target_path);
                }
            }
            catch (const std::filesystem::filesystem_error &e)
            {
                std::cerr << "Error copying " << path << ": " << e.what() << std::endl;
            }
        }
    }

    /**
     * @struct RosLoadConfig
     * @ingroup group_interface
     * @brief Utility struct to load configuration and log paths for ROS nodes.
     * @details This struct helps in constructing the configuration folder path based on the package name and
     * config folder name. It also constructs a log path in the user's home directory, optionally
     * appending a timestamp if the LOG_FOLDER_TIMESTAMP environment variable is set.
     * It ensures that the log directory exists, creating it if necessary.
     */
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

            CopyFilesRecursively(config_folder, log_path);
        }
    };
} // namespace romoco
#endif // BIPED_ROS_ROS_LOAD_CONFIG_HPP