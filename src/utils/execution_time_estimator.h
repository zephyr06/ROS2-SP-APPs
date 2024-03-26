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
        std::vector<std::string> node_vec{"tsp", "mpc", "rrt", "slam"};
        YAML::Node statitics_node;

        for (int i = 0; i < node_vec.size(); i++)
        {
            auto node_name = node_vec[i];
            // Open the file
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
                data.push_back(number);
            }

            // Close the file
            file.close();

            if (data.empty())
            {
                std::cerr << "No data found in the file" << std::endl;
                return;
            }

            // Only keep the lastest max_data_count entries
            if (data.size() > max_data_count)
            {
                data.erase(data.begin(), data.end() - max_data_count);
            }

            // Calculate statistics
            double mean, std_dev, min_val, max_val;
            calculateStatistics(data, mean, std_dev, min_val, max_val);

            // Output results to YAML file
            YAML::Node yaml_node;
            yaml_node["id"] = i;
            yaml_node["execution_time_mu"] = mean;
            yaml_node["execution_time_sigma"] = std_dev;
            yaml_node["execution_time_min"] = min_val;
            yaml_node["execution_time_max"] = max_val;
            yaml_node["period"] = max_val;
            yaml_node["deadline"] = max_val;

            std::string my_str = node_name;
            transform(my_str.begin(), my_str.end(), my_str.begin(), ::toupper);
            yaml_node["name"] = my_str;

            statitics_node["Tasks"].push_back(yaml_node);
        }

        // Output YAML node to a file
        std::ofstream output_file(getTimeRecordFolder() + "execution_time_statistics.yaml");
        output_file << statitics_node;
        output_file.close();

        std::cout << "Results have been written to output.yaml" << std::endl;

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
    }
};