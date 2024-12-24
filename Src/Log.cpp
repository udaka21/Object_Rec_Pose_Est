/**
* @file Log.cpp
* @author Udaka Ayas Manawadu
* @date 2024-05-20
* @brief Implementation of the Log class
**/

#include "Log.h"

Log::Log(const std::string& filename) : logLevel(INFO)
{
    logFile.open(filename, std::ios_base::app);
    if (!logFile.is_open())
    {
        std::cerr << "Unable to open log file: " << filename << std::endl;
    }
}

Log::~Log()
{
    if (logFile.is_open())
    {
        logFile.close();
    }
}

Log& Log::getInstance(const std::string& filename)
{
    static Log instance(filename);
    return instance;
}

void Log::log(Level level, const std::string& message)
{
    std::lock_guard<std::mutex> guard(logMutex);
    if (level <= logLevel && logFile.is_open())
    {
        std::string logMessage = "[" + getCurrentTime() + "] [" + levelToString(level) + "] " + message;
        logFile << logMessage << std::endl;
        std::cout << logMessage << std::endl; // Also log to the console
    }
}

void Log::setLevel(Level level)
{
    logLevel = level;
}

std::string Log::getCurrentTime()
{
    std::time_t now = std::time(nullptr);
    std::tm* tmPtr = std::localtime(&now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", tmPtr);
    return buffer;
}

std::string Log::levelToString(Level level)
{
    switch (level)
    {
    case ERROR: return "ERROR";
    case WARNING: return "WARNING";
    case INFO: return "INFO";
    case DEBUG: return "DEBUG";
    default: return "UNKNOWN";
    }
}

void Log::createCSV(const std::string& filename)
{
    std::ofstream csvFile(filename);
    if (!csvFile.is_open())
    {
        std::cerr << "Unable to create CSV file: " << filename << std::endl;
    }
    else
    {
        //Write the header
        csvFile << "Variable Name, Data" << std::endl;

        csvFile.close();
    }
}

void Log::addDataToCSV(const std::string& filename, const std::string& data_name, const std::string& message)
{
    std::lock_guard<std::mutex> guard(logMutex);
    std::ofstream csvFile(filename, std::ios_base::app);
    if (!csvFile.is_open())
    {
        std::cerr << "Unable to open CSV file: " << filename << std::endl;
    }
    else
    {
        // Write the log message in CSV format
        csvFile << data_name << "," << message << std::endl;
        csvFile.close();
    }
}
