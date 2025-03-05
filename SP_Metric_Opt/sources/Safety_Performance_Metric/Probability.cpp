#include "sources/Safety_Performance_Metric/Probability.h"

#include "algorithm"
namespace SP_OPT_PA {

#if defined(RYAN_HE_CHANGE)
// Function to generate a random value from a Gaussian distribution
double getRandomValueByMuSigma(double mu, double std, double* minVal,
                               double* maxVal) {
    // Random number generator
    static std::random_device rd;           // Seed for the random number engine
    static std::mt19937 gen(rd());          // Mersenne Twister engine
    std::normal_distribution<> d(mu, std);  // Gaussian distribution

    double randomValue = d(gen);  // Generate and return a random value
    if (minVal != nullptr && randomValue < *minVal)
        randomValue = *minVal;
    if (maxVal != nullptr && randomValue > *maxVal)
        randomValue = *maxVal;
    return randomValue;
}
#endif

std::unordered_map<double, double> FiniteDist::GetV_PMap() const {
    std::unordered_map<double, double> m;
    for (const Value_Proba& element : distribution)
        m[element.value] = element.probability;
    return m;
}
bool Value_Proba::operator==(const Value_Proba& other) const {
    double tolerance = 1e-1;
    return approx_equal_double(value, other.value, tolerance) &&
           approx_equal_double(probability, other.probability, tolerance);
}

FiniteDist::FiniteDist(const GaussianDist& gauss_dist, double min_val,
                       double max_val, int granularity)
    : min_time(min_val), max_time(max_val) {
    distribution.reserve(granularity);
    if (granularity < 1)
        CoutError("Invalid granularity!");

    double step = (max_val - min_val) / (double(granularity) - 1);
    distribution.push_back(Value_Proba(min_val, gauss_dist.CDF(min_val)));
    if (step > 0) {
        for (double execution_time = min_val + step; execution_time < max_val;
             execution_time += step) {
            double chance = gauss_dist.CDF(execution_time) -
                            gauss_dist.CDF(execution_time - step);
            distribution.push_back(Value_Proba(execution_time, chance));
        }

        double sum_probability_added = 0;
        for (const Value_Proba& vp : distribution)
            sum_probability_added += vp.probability;
        distribution.push_back(
            Value_Proba(max_val, 1.0 - sum_probability_added));
    } else {
        distribution = {Value_Proba(min_val, 1.0)};
    }
    CheckDistributionValid();
}

// O(n log(n))
void FiniteDist::UpdateDistribution(
    const std::unordered_map<double, double>& m_v2p) {
    distribution.clear();
    distribution.reserve(m_v2p.size());
    for (auto itr = m_v2p.begin(); itr != m_v2p.end(); itr++) {
        distribution.push_back(Value_Proba(itr->first, itr->second));
    }

    SortDist();
    UpdateMinMaxValues();
}

// O(n log(n))
void FiniteDist::Coalesce(const FiniteDist& other) {
    std::unordered_map<double, double> m_v2p = GetV_PMap();
    for (const Value_Proba& element : other.distribution) {
        if (m_v2p.count(element.value))
            m_v2p[element.value] += element.probability;
        else
            m_v2p[element.value] = element.probability;
    }
    UpdateDistribution(m_v2p);
}

// O(n^2)
void FiniteDist::Convolve(const FiniteDist& other) {
    std::unordered_map<double, double> m_v2p;
    for (const auto& element_this : distribution) {
        for (const auto& element_other : other.distribution) {
            double value = element_this.value + element_other.value;
            double prob = element_this.probability * element_other.probability;
            if (m_v2p.count(value))
                m_v2p[value] += prob;
            else
                m_v2p[value] = prob;
        }
    }
    UpdateDistribution(m_v2p);
}

std::vector<Value_Proba> FiniteDist::GetTailDistribution(double preempt_time) {
    auto itr =
        std::upper_bound(distribution.begin(), distribution.end(), preempt_time,
                         [](double preempt_time, const Value_Proba& element) {
                             return preempt_time < element.value;
                         });
    if (itr == distribution.end())
        return {};
    else {
        std::vector<Value_Proba> tail_dist(itr, distribution.end());
        return tail_dist;
    }
}

std::vector<Value_Proba> FiniteDist::GetHeadDistribution(size_t tail_size) {
    std::vector<Value_Proba> head_dist(
        distribution.begin(), distribution.begin() + size() - tail_size);
    return head_dist;
}

// return whether new preemption is added
bool FiniteDist::AddOnePreemption(const FiniteDist& execution_time_dist,
                                  double preempt_time) {
    std::vector<Value_Proba> tail_dist = GetTailDistribution(preempt_time);
    if (tail_dist.size() == 0)
        return false;
    else {
        FiniteDist head(GetHeadDistribution(tail_dist.size()));
        FiniteDist tail(tail_dist);
        tail.Convolve(execution_time_dist);
        head.Coalesce(tail);
        UpdateDistribution(head.distribution);
        CompressDistribution(int(GlobalVariables::Granularity * 1.0),
                             GlobalVariables::Dist_compress_threshold);
        return true;
    }
}
void FiniteDist::CompressDeadlineMissProbability(double deadline) {
    auto itr_exceed_ddl =
        std::upper_bound(distribution.begin(), distribution.end(), deadline,
                         [](double deadline, const Value_Proba& element) {
                             return deadline < element.value;
                         });
    double ddl_miss = 0;
    for (auto itr = itr_exceed_ddl; itr != distribution.end(); itr++) {
        ddl_miss += itr->probability;
    }
    distribution.erase(itr_exceed_ddl, distribution.end());
    if (ddl_miss > 0)
        distribution.push_back(Value_Proba(deadline + 1, ddl_miss));
}
void FiniteDist::AddPreemption(const FiniteDist& execution_time_dist_hp,
                               double period_hp, double deadline_this) {
    // backlog
    Convolve(execution_time_dist_hp);
    int hp_instance_considered = 1;
    while (max_time <= deadline_this &&
           max_time > hp_instance_considered * period_hp) {
        AddOnePreemption(execution_time_dist_hp,
                         hp_instance_considered * period_hp);
        hp_instance_considered++;
    }
    CompressDeadlineMissProbability(deadline_this);
}

inline bool near(double a, double b) { return abs(a - b) < 1e-6; }

/*
Implementation consideration:
If data_raw is in a small range, actual granularity can be obtained by evenly
dividing the range;
- Always create a separate value-probability pair for the very large data (e.g.,
1e9 and its probaility)
- After excluding the very large data, if the data is still spreaded within a
large range, we increase the actual granularity to ensure precision

TODO: consider improve the run-time speed
*/
FiniteDist::FiniteDist(const std::vector<double>& data_raw,
                       int granularity_suggest) {
    long long unschedulable_max_time = 1e11 - 1;
    int data_size = data_raw.size();
    double granularity = granularity_suggest;
    std::vector<double> data = data_raw;
    std::sort(data.begin(), data.end());
    auto itr =
        std::lower_bound(data.begin(), data.end(), unschedulable_max_time);

    int max_index_during_iterate = data_size - 1;
    if (itr != data.end()) {
        distribution.push_back(
            Value_Proba(*itr, double(int(data.end() - itr)) / data_size));
    }
    data.erase(itr, data.end());
    if (data.size() > 0) {
        min_time = data[0];
        max_time = data[data.size() - 1];
        // if (max_time / min_time > 10) {
        granularity =
            int(std::max(granularity, double(max_time / min_time) * 5));
        // }
        double range = max_time - min_time;
        double step = range / (granularity - 1);

        distribution.reserve(granularity);
        int value_index = 0;
        for (int i = 0; i < granularity; i++) {
            double val = min_time + i * step;
            int count = 0;
            while (value_index < static_cast<int>(data_size) &&
                   data[value_index] - 1e-3 <= val) {
                count++;
                value_index++;
            }
            if (count > 0)
                distribution.push_back(
                    Value_Proba(val, double(count) / data_size));
        }
    }
    CheckDistributionValid();
    UpdateDistribution(distribution);
}

void FiniteDist::CheckDistributionValid() const {
    double sum = 0;
    for (int i = 0; i < distribution.size(); i++) {
        sum += distribution[i].probability;
    }
    if (abs(sum - 1.0) > 1e-4) {
        CoutError("Error in FiniteDist constructor: sum of probabilities is " +
                  std::to_string(sum));
    }
}

double FiniteDist::CDF(double x) const {
    double cdf = 0;
    for (const Value_Proba& element : distribution) {
        if (element.value <= x)
            cdf += element.probability;
        else
            break;
    }
    return cdf;
}

// start_index and end_index are inclusive
void CompressDistributionVector(std::vector<Value_Proba>& vec, int start_index,
                                int end_index, int size_after_compres) {
    int element_counts_before_merge = end_index - start_index + 1;
    if (element_counts_before_merge <= size_after_compres)
        return;

    int size_per_merge =
        ceil(double(element_counts_before_merge) / size_after_compres);
    std::vector<Value_Proba> new_vec;
    if (size_after_compres < 0)
        CoutError(
            "Error in CompressDistributionVector: size_after_compres is "
            "negative");
    new_vec.reserve(size_after_compres);
    double sum = 0;
    int count = 0;
    for (int i = start_index; i <= end_index; i++) {
        sum += vec[i].probability;
        count++;
        if (count == size_per_merge) {
            new_vec.push_back(Value_Proba(vec[i].value, sum));
            sum = 0;
            count = 0;
        }
    }
    if (count > 0) {
        new_vec.push_back(Value_Proba(vec[end_index].value, sum));
    }
    vec.erase(vec.begin() + start_index, vec.begin() + end_index + 1);
    vec.insert(vec.begin() + start_index, new_vec.begin(), new_vec.end());
}

void FiniteDist::CompressDistribution(size_t max_size,
                                      double compress_threshold) {
    if (distribution.size() <= max_size)
        return;
    int compress_index_since = distribution.size();
    for (size_t i = 0; i < distribution.size(); i++) {
        if (distribution[i].probability < compress_threshold) {
            compress_index_since = i;
            break;
        }
    }

    if (compress_index_since == distribution.size())
        return;

    CompressDistributionVector(
        distribution, compress_index_since, distribution.size() - 1,
        std::max(static_cast<int>(max_size) - compress_index_since, 1));
}

void FiniteDist::CompressDistributionWithOnlySize(size_t max_size) {
    CompressDistribution(max_size, 1.0 / max_size);
}

double FiniteDist::GetAvgValue() const {
    if (avg_time > 0)
        return avg_time;
    else {
        double sum = 0;
        for (const Value_Proba& vp : distribution) {
            sum += vp.value * vp.probability;
        }
        return sum;
    }
}
bool FiniteDist::approx_equal(const FiniteDist& other, double tolerance) const {
    if (distribution.size() != other.distribution.size())
        return false;
    for (uint i = 0; i < distribution.size(); i++) {
        if (distribution[i] != other.distribution[i])
            return false;
    }
    if (near(min_time, other.min_time) && near(max_time, other.max_time))
        return true;
    return false;
}

bool FiniteDist::operator==(const FiniteDist& other) const {
    if (distribution.size() != other.distribution.size())
        return false;
    double tolerance = 1e-1;
    for (uint i = 0; i < distribution.size(); i++) {
        if (distribution[i] != other.distribution[i])
            return false;
    }
    if (!approx_equal_double(min_time, other.min_time, tolerance))
        return false;
    if (!approx_equal_double(max_time, other.max_time, tolerance))
        return false;
    return true;
}
}  // namespace SP_OPT_PA