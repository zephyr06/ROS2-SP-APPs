#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <cctype>

#include <yaml-cpp/yaml.h>
#include "profiler.h"

class ExecutionTimeEstimator
{
public:
    ExecutionTimeEstimator() {}

    void updateTaskExecutionTimeDistributions(int max_data_count = 50)
    {
        // update period and deadlines
        std::string task_characteristics_yaml = getTimeRecordFolder() + "task_characteristics.yaml";
        YAML::Node tasks = YAML::LoadFile(task_characteristics_yaml);

        int time_scale_multiplier = 1000;
        YAML::Node statitics_node;

        for (YAML::Node::iterator it_task = tasks["tasks"].begin(); it_task != tasks["tasks"].end(); ++it_task)
        {
            auto node_name = it_task->operator[]("name").as<std::string>();

            // Open the file
            transform(node_name.begin(), node_name.end(), node_name.begin(), ::tolower);
            std::string filename = getTimeRecordFolder() + node_name + "_execution_time.txt";
            std::ifstream file(filename);
            if (!file.is_open())
            {
                std::cerr << "Error opening file" << std::endl;
                return;
            }

            // Read data and extract numbers
            std::vector<double> data;
            std::string line;
            while (std::getline(file, line))
            {
                double number = extractNumber(line);
                if (number <= 0)
                    continue;

                data.push_back(number * time_scale_multiplier);
            }

            // Close the file
            file.close();

            if (data.empty())
            {
                std::cerr << "No data found in the file" << std::endl;
                return;
            }

            // Only keep the lastest max_data_count entries
            if ((int)data.size() > max_data_count)
            {
                data.erase(data.begin(), data.end() - max_data_count);
            }

            // Calculate statistics
            double mean, std_dev, min_val, max_val;
            calculateStatistics(data, mean, std_dev, min_val, max_val);

            // Modify YAML file
            it_task->operator[]("execution_time_mu") = mean;
            it_task->operator[]("execution_time_sigma") = std_dev;
            it_task->operator[]("execution_time_min") = min_val;
            it_task->operator[]("execution_time_max") = max_val;
        }

        // Output YAML node to a file
        std::ofstream output_file(getTimeRecordFolder() + "task_characteristics.yaml");
        output_file << tasks;
        output_file.close();

        return;
    }

private:
    // Function to extract the last number after "::"
    double extractNumber(const std::string &line)
    {
        std::size_t pos = line.find_last_of("::");
        if (pos != std::string::npos)
        {
            return std::stod(line.substr(pos + 1));
        }
        return 0.0; // Default value if "::" is not found
    }

    // Function to calculate statistics
    void calculateStatistics(const std::vector<double> &data, double &mean, double &std_dev, double &min_val, double &max_val)
    {
        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        mean = sum / data.size();

        double sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
        std_dev = std::sqrt(sq_sum / data.size() - mean * mean);

        min_val = *std::min_element(data.begin(), data.end());
        max_val = *std::max_element(data.begin(), data.end());

        double min_threshold = 1e-3;
        mean = std::max(mean, min_threshold);
        std_dev = std::max(std_dev, min_threshold);
        min_val = std::max(min_val, min_threshold);
        max_val = std::max(max_val, min_threshold);
    }
};