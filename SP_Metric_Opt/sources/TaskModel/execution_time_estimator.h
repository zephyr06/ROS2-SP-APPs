#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

// Function to extract the last number after "::"
double extractNumber(const std::string &line) {
    std::size_t pos = line.find_last_of("::");
    if (pos != std::string::npos) {
        return std::stod(line.substr(pos + 1));
    }
    return 0.0;  // Default value if "::" is not found
}
// Function to extract the last number after "::"
double extractIndex(const std::string &line) {
    std::size_t pos_double_colon = line.find_last_of("::");
    if (pos_double_colon != std::string::npos) {
        std::string first_part = line.substr(0, pos_double_colon);
        std::size_t pos_space = first_part.find_last_of(" ");
        return std::stod(first_part.substr(pos_space + 1));
    }
    return 0.0;  // Default value if "::" is not found
}

std::vector<std::string> GetAllLines(std::string file_name) {
    std::ifstream file(file_name);
    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) {
        lines.push_back(line);
    }
    file.close();
    return lines;
}

// return a vector of execution time data and the miss value rate
std::pair<std::vector<double>, int> ReadExtTimeData(std::string file_name,
                                                    int last_data_count) {
    std::vector<std::string> lines = GetAllLines(file_name);

    std::vector<double> data;
    data.reserve(last_data_count);

    int n = lines.size();
    for (int i = std::max(n - last_data_count, 0); i < n; i++) {
        std::string line = lines[i];
        double number = extractNumber(line);
        if (number <= 0) continue;
        data.push_back(number);
    }
    int miss_count = 0;
    int last_index = extractIndex(lines[n - 1]);
    int first_index;
    if (n >= last_data_count) {
        first_index = extractIndex(lines[n - last_data_count]);
        miss_count = (last_index - first_index - last_data_count + 1);
    } else {
        first_index = extractIndex(lines[0]);
        miss_count = (last_index - first_index - n + 1);
    }
    return {data, miss_count};
}

void FillMissValue(std::vector<double> &data, int miss_count, int miss_val) {
    data.reserve(data.size() + miss_count);
    for (int i = 0; i < miss_count; i++) {
        data.push_back(miss_val);
    }
}