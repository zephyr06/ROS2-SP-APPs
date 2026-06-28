#include "Parameters.h"

#include <yaml-cpp/yaml.h>

namespace GlobalVariables {
const std::string PROJECT_PATH = std::string(SP_OPT_PROJECT_ROOT_DIR) + "/";
YAML::Node loaded_doc =
    YAML::LoadFile(PROJECT_PATH + "sources/parameters.yaml");

// optimization settings
int debugMode = loaded_doc["debugMode"].as<int>();
int TIME_LIMIT = loaded_doc["TIME_LIMIT"].as<int>();
int printRTA = loaded_doc["printRTA"].as<int>();

int Granularity = loaded_doc["Granularity"].as<int>();
int Layer_Node_During_Incremental_Optimization =
    loaded_doc["Layer_Node_During_Incremental_Optimization"].as<int>();
double Dist_compress_threshold =
    loaded_doc["Dist_compress_threshold"].as<double>();
bool disable_time_limit_opt = false;
bool use_wcet_execution_time = false;
int TimeLimitSearchRadiusIncr =
    loaded_doc["TimeLimitSearchRadiusIncr"].as<int>();

// simulation export controls
int EXPORT_DETAIL_LEVEL = 3;               // default to FULL for backward compatibility
int METRIC_SAMPLE_INTERVAL_SECONDS = 0;    // default to 0 (all intervals)

// --- optional YAML overrides with fallback defaults ---
// These run before main() because they are in the same namespace as the
// variables above and depend on `loaded_doc` having already loaded.
static bool _exportDefaultsSet = []() {
    try {
        EXPORT_DETAIL_LEVEL =
            loaded_doc["EXPORT_DETAIL_LEVEL"].as<int>();
    } catch (const YAML::Exception&) {
        // keep hard-coded default
    }
    try {
        METRIC_SAMPLE_INTERVAL_SECONDS =
            loaded_doc["METRIC_SAMPLE_INTERVAL_SECONDS"].as<int>();
    } catch (const YAML::Exception&) {
        // keep hard-coded default
    }
    return true;
}();

}  // namespace GlobalVariables
