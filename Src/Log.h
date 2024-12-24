/**
* @file Log.h
 * @author Udaka Ayas Manawadu
 * @date 2024-05-20
 * @license BSD 2-Clause License
 *
 * @brief This header file defines the `Log` class.
 *
 * The `Log` class provides a thread-safe logging system for applications.
 * It supports multiple logging levels (ERROR, WARNING, INFO, DEBUG),
 * as well as the ability to create and append data to CSV files.
 */

#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <mutex>

class Log
{
public:
    /**
     * @enum Level
     * @brief Represents the logging levels.
     */
    enum Level
    {
        ERROR, ///< Critical errors requiring immediate attention.
        WARNING, ///< Non-critical issues that may need investigation.
        INFO, ///< General informational messages.
        DEBUG ///< Detailed debugging information.
    };

    /**
     * @brief Gets the singleton instance of the Log class.
     * @param filename The name of the log file (default: "default.log").
     * @return Reference to the singleton instance.
     */
    static Log& getInstance(const std::string& filename = "default.log");

    /**
     * @brief Logs a message with a specified logging level.
     * @param level The logging level.
     * @param message The message to log.
     */
    void log(Level level, const std::string& message);

    /**
     * @brief Sets the logging level for the logger.
     * @param level The logging level to set.
     */
    void setLevel(Level level);

    /**
     * @brief Gets the current time as a string.
     * @return Current time in the format "YYYY-MM-DD HH:MM:SS".
     */
    static std::string getCurrentTime();

    /**
     * @brief Creates a CSV file with a header.
     * @param filename The name of the CSV file to create.
     */
    void createCSV(const std::string& filename);

    /**
     * @brief Appends data to a CSV file.
     * @param filename The name of the CSV file.
     * @param data_name The name of the data.
     * @param message The data to log.
     */
    void addDataToCSV(const std::string& filename, const std::string& data_name, const std::string& message);

private:
    Log(const std::string &filename); ///< Private constructor for singleton.
    ~Log(); ///< Destructor to close the log file.
    Log(const Log&) = delete; ///< Deleted copy constructor.
    Log& operator=(const Log&) = delete; ///< Deleted assignment operator.

    std::ofstream logFile; ///< File stream for logging.
    Level logLevel; ///< Current logging level.
    std::mutex logMutex; ///< Mutex for thread-safe logging.

    /**
     * @brief Converts a logging level to its string representation.
     * @param level The logging level.
     * @return The string representation of the level.
     */
    std::string levelToString(Level level);
};

#endif // LOG_H
