#pragma once
#include <cmath>
#include <unordered_map>
#include <vector>

#include "sources/Utils/Parameters.h"
#include "sources/Utils/testMy.h"

#if defined(RYAN_HE_CHANGE)
#include <random>
#endif


namespace SP_OPT_PA {

#if defined(RYAN_HE_CHANGE)
// Function to generate a random value from a Gaussian distribution
double getRandomValueByMuSigma(double mu, double std, double *minVal, double *maxVal);
#endif

class ProbabilityDistributionBase {
   public:
    ProbabilityDistributionBase() {}

    double CDF(double x) const;

    // virtual void UpdateDistribution(std::vector<double> execution_time_data);

    // data members
};


class GaussianDist : public ProbabilityDistributionBase {
   public:
    GaussianDist() {}
    GaussianDist(double mu, double sigma) : mu(mu), sigma(sigma) {
        if (abs(sigma) < 1e-6) CoutError("Invalid sigma value!");
    }

    inline double CDF(double x) const {
        double z = (x - mu) / (sigma * std::sqrt(2.0));
        return 0.5 * (1.0 + std::erf(z));
    }
    inline bool operator==(const GaussianDist& other) const {
        return mu == other.mu && sigma == other.sigma;
    }
    inline bool operator!=(const GaussianDist& other) const {
        return !(*this == other);
    }

    // data
    double mu;
    double sigma;
};

inline bool approx_equal_double(double a, double b, double tolerance_rel) {
    if (a == 0 && b == 0) return true;
    if (a == 0 || b == 0) return false;
    return std::abs(a - b) / std::abs(a) < tolerance_rel;
}
struct Value_Proba {
    Value_Proba() {}
    Value_Proba(double v, double p) : value(v), probability(p) {}

    bool operator==(const Value_Proba& other) const;

    inline bool operator!=(const Value_Proba& other) const {
        return !(*this == other);
    }

    double value;
    double probability;
};

// start_index and end_index are inclusive
void CompressDistributionVector(std::vector<Value_Proba>& vec, int start_index,
                                int end_index, int size_after_compres);

// stores a probability mass function, i.e., value-probability pair
// this should be the major way to describe probability distribution
class FiniteDist : public ProbabilityDistributionBase {
   public:
    FiniteDist() {}
    FiniteDist(const GaussianDist& gauss_dist, double min_val, double max_val,
               int granularity);

    FiniteDist(const GaussianDist& gauss_dist, int granularity)
        : FiniteDist(gauss_dist, gauss_dist.mu - 2.3263 * gauss_dist.sigma,
                     gauss_dist.mu + 2.3263 * gauss_dist.sigma, granularity) {}

    FiniteDist(const std::vector<Value_Proba>& distribution_input) {
        UpdateDistribution(distribution_input);
    }

    FiniteDist(const std::vector<double>& data_raw, int granularity);

#if defined(RYAN_HE_CHANGE)
    double getMinValue() const {
        return min_time;
    }
    double getMaxValue() const {
        return max_time;
    }
    // get a random value following the distribution
    double getRandomValue() {
        if (distribution.empty()) {
            throw std::runtime_error("Distribution is empty. Cannot generate random value.");
        }

        // Generate a random number between 0 and 1
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        double random_probability = dis(gen);

        // Iterate through the distribution and select the corresponding value
        double cumulative_probability = 0.0;
        for (size_t i = 0; i < distribution.size(); ++i) {
            cumulative_probability += distribution[i].probability;

            // If random_probability falls within this bin
            if (random_probability <= cumulative_probability) {
                // If it's the first bin, return the first value
                if (i == 0) {
                    return distribution[i].value;
                }

                // Otherwise, interpolate between the current bin and the previous bin
                double prev_cumulative = cumulative_probability - distribution[i].probability;
                double weight = (random_probability - prev_cumulative) / distribution[i].probability;

                // Linear interpolation between the two values
                return distribution[i - 1].value + weight * (distribution[i].value - distribution[i - 1].value);
            }
        }

        // If no value is found (due to rounding errors), return the last value
        return distribution.back().value;
    }
#endif

    void CheckDistributionValid() const;
    void UpdateDistribution(
        const std::vector<Value_Proba>& distribution_input) {
        distribution = distribution_input;
        SortDist();
        UpdateMinMaxValues();
    }

    void UpdateMinMaxValues() {
        if (size() > 0) {
            min_time = distribution[0].value;
            max_time = distribution.back().value;
        }
    }

    inline size_t size() const { return distribution.size(); }

    inline Value_Proba& operator[](size_t i) { return distribution[i]; }
    inline const Value_Proba& at(size_t i) { return distribution.at(i); }

    // ''merge'' two distribution
    void Coalesce(const FiniteDist& other);

    // standard convolution to perform probabilitistic addition
    void Convolve(const FiniteDist& other);

    void Normalize() {
        int p_sum = 0;
        for (const Value_Proba& element : distribution)
            p_sum += element.probability;
        for (Value_Proba& element : distribution) element.probability /= p_sum;
    }

    std::unordered_map<double, double> GetV_PMap() const;

    // sort distribution
    inline void SortDist() {
        std::sort(distribution.begin(), distribution.end(),
                  [](const Value_Proba& dis1, const Value_Proba& dis2) {
                      return dis1.value < dis2.value;
                  });
    }
    void UpdateDistribution(const std::unordered_map<double, double>& m_v2p);

    std::vector<Value_Proba> GetTailDistribution(double preempt_time);
    std::vector<Value_Proba> GetHeadDistribution(size_t tail_size);

    bool AddOnePreemption(const FiniteDist& execution_time_dist,
                          double preempt_time);
    void AddPreemption(const FiniteDist& execution_time_dist_hp,
                       double period_hp, double dealine_this);
    void CompressDeadlineMissProbability(double deadline);

    void print() const {
        for (const auto vp : distribution)
            std::cout << "(" << vp.value << ", " << vp.probability << ") ";
        std::cout << "\n";
    }

    void Scale(double k) {
        for (Value_Proba& pair : distribution) pair.value *= k;
    }
    bool approx_equal(const FiniteDist& other, double tolerance) const;
    bool approx_not_equal(const FiniteDist& other, double tolerance) const {
        return !approx_equal(other, tolerance);
    }
    bool operator==(const FiniteDist& other) const;
    bool operator!=(const FiniteDist& other) const {
        return !((*this) == other);
    }

    double CDF(double x) const;

    void CompressDistribution(size_t max_size, double compress_threshold);

    double GetAvgValue() const;

    // data members
    // saves the probability that x<= value, this is probability mass function
    // rather than cumulative function
    std::vector<Value_Proba> distribution;
    double min_time;
    double max_time;
    double avg_time = -1;
};

inline FiniteDist GetUnitExecutionTimeDist(double time_limit) {
    std::vector<Value_Proba> v_p = {{time_limit, 1.0}};
    return FiniteDist(v_p);
}
}  // namespace SP_OPT_PA