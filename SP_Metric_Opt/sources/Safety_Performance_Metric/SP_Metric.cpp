
#include "sources/Safety_Performance_Metric/SP_Metric.h"

#include "sources/Utils/readwrite.h"
#include "sources/Utils/Parameters.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>

namespace SP_OPT_PA {

#if defined(RYAN_HE_CHANGE)
// RYAN_HE: moved from .h to .cpp
// just want to avoid keep recompiling eveything when anything changes in .h

// violate_probability is deadline miss chance
// threshold is deadline miss threshold
//
// PenaltyFunc: deadline miss chance > threshold
//     -0.01 * exp(10 * abs(threshold - violate_probability));
// RewardFunc:  deadline miss chance <= threshold
//   log((threshold - violate_probability) + 1);
double SP_Func(double violate_probability, double threshold) {
    double min_val, max_val, val;
    min_val = PenaltyFunc(1, threshold); // smallest value, 100% miss
    max_val = RewardFunc(0, threshold);  // largest value, 0% miss
    if (threshold >= violate_probability) {
        val = RewardFunc(violate_probability, threshold);
    } else {
        val = PenaltyFunc(violate_probability, threshold);
    }
    // normalize
    return interpolate(val, min_val, 0, max_val, 1);
}
#endif

std::vector<double> GetChainsDDL(const DAG_Model& dag_tasks) {
    // std::vector<double> chains_ddl(dag_tasks.chains_.size(),
    //                                HyperPeriod(dag_tasks.tasks));
    return dag_tasks.chains_deadlines_;
}
double ObtainSP(const FiniteDist& dist, double deadline,
                double ddl_miss_threshold, double weight) {
    double ddl_miss_chance = GetDDL_MissProbability(dist, deadline);
    return SP_Func(ddl_miss_chance, ddl_miss_threshold) * weight;
}

// timePerformancePairs is required to be sorted by time
double GetPerfTerm(const std::vector<TimePerfPair>& timePerformancePairs,
                   double time_limit) {
    // Find the two time-performance pairs that surround the given time limit
    auto it = std::upper_bound(timePerformancePairs.begin(),
                               timePerformancePairs.end(), time_limit,
                               [](double time, const TimePerfPair& pair) {
                                   return time < pair.time_limit;
                               });

    if (it == timePerformancePairs.begin()) {
        // The given time limit is smaller than the smallest time in the pairs
        return 0.0;
    } else if (it == timePerformancePairs.end()) {
        // The given time limit is larger than the largest time in the pairs
        return timePerformancePairs.back().performance;
    } else {
        // The given time limit is between two time-performance pairs
        auto prevPair = std::prev(it);
        return interpolate(time_limit, prevPair->time_limit,
                           prevPair->performance, it->time_limit,
                           it->performance);
    }
}
// double ObtainSP(const std::vector<FiniteDist>& dists,
//                 const std::vector<double>& deadline,
//                 const std::unordered_map<int, double>& ddl_miss_thresholds,
//                 const std::unordered_map<int, double>& weights) {
//     int n = dists.size();
//     double sp_overall = 0;
//     for (int i = 0; i < n; i++) {
//         double ddl_miss_chance = GetDDL_MissProbability(dists[i],
//         deadline[i]); sp_overall +=
//             SP_Func(ddl_miss_chance, ddl_miss_thresholds[i]) * weights[i];
//     }
//     return sp_overall;
// }

#define TMP_DBG

double ObtainSP_TaskSet(const TaskSet& tasks,
                        const SP_Parameters& sp_parameters) {
                      
    std::vector<FiniteDist> rtas = ProbabilisticRTA_TaskSet(tasks);
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric) {
        std::cout << "####ObtainSP_TaskSet: ProbabilisticRTA_TaskSet DONE. "<<std::endl;
        for (int i = 0; i < rtas.size(); i++) {
            std::cout << "####ObtainSP_TaskSet: rtas["<<i<<"]: "<<std::endl;
            rtas[i].print();
        }
    }
#endif      
    // std::vector<double> deadlines = GetParameter<double>(tasks, "deadline");
    // return ObtainSP(rtas, deadlines, sp_parameters.thresholds_node,
    //                 sp_parameters.weights_node);
    double sp_overall = 0;
    
    // FOR DEBUG PRINT
    std::vector<double> ddl_miss_chances;
    
    //std::cout<<std::endl;
    for (int i = 0; i < tasks.size(); i++) {
        int task_id = tasks[i].id;
        double ddl_miss_chance =
            GetDDL_MissProbability(rtas[i], tasks[i].deadline);
        
        ddl_miss_chances.push_back(ddl_miss_chance);

#if defined(RYAN_HE_CHANGE_DEBUG)
        double weight = sp_parameters.weights_node.at(task_id);
        double sp_threshold = sp_parameters.thresholds_node.at(task_id);
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric) {
            std::cout << "####ObtainSP_TaskSet: task_id="<<task_id<<", deadline="<<tasks[i].deadline<<", prd="<<tasks[i].period;
            std::cout<<", ddl_miss_chance="<<ddl_miss_chance<<", weight="<<weight;
            std::cout<<", sp_threshold="<<sp_threshold<<", SP_Func ... "<<std::endl;
        }
        double sp_this = SP_Func(ddl_miss_chance, sp_threshold) * weight;
        sp_overall += sp_this;
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric)
            std::cout << "####ObtainSP_TaskSet: task_id = "<<task_id<<", sp_this = "<<sp_this<<std::endl;
#else          
        sp_overall += SP_Func(ddl_miss_chance,
                              sp_parameters.thresholds_node.at(task_id)) *
                              sp_parameters.weights_node.at(task_id);
#endif
        // RYAN_20250308
        // if ddl_miss_chance is 1.0, then sp_loss for this task will be same, no matter
        // what is the execution time selected 
        // however, to break tie, we still want to select the one with smaller execution time
        if (ddl_miss_chance>=0.99999) {
            double v = tasks[i].execution_time_dist.GetAvgValue();
            //printf("-------- Ryan %s task%d, reduce SP further for ddl_miss==1, EtAvg=%.4f, deadline=%f, sp=%f\n",
            //    __func__,i,v,tasks[i].deadline,sp_overall);
            sp_overall -= v/tasks[i].deadline*0.001;
            // printf("sp=%f\n",sp_overall);
        }
    }

    if (GlobalVariables::debugMode & DBG_PRT_MSK_DBG_DDL_SP) {
        int n = ddl_miss_chances.size();
        if (ddl_miss_chances[n-1] > 0.3) {
            printf("DBG_PRT_MSK_DBG_DDL_SP: %s: ddl_miss_chances = ",__func__);
            for (int i = 0; i < n; i++) {
                printf("%.2f ",ddl_miss_chances[i]);
            }
            printf("\n");

            printf("DBG_PRT_MSK_DBG_DDL_SP: %s: tasks (id,deadline,et) ...\n",__func__);
            for (int i = 0; i < tasks.size(); i++) {
                printf("(%d,%.0f,%.2f) ",
                    tasks[i].id,tasks[i].deadline,tasks[i].execution_time_dist.GetAvgValue());
            }
            printf("\n");

            printf("DBG_PRT_MSK_DBG_DDL_SP: %s:rtas ...\n",__func__);
            for (int i = 0; i < rtas.size(); i++) {
                if (ddl_miss_chances[i]>0.0) {
                    std::cout << "rtas["<<i<<"]: ";
                    rtas[i].print();
                }
            }
        }
    }

    // printf("Ryan %s this sp_overall = %f\n",__func__,sp_overall);
    return sp_overall;
}

double ObtainSP_DAG(const DAG_Model& dag_tasks,
                    const SP_Parameters& sp_parameters) {
    if (GlobalVariables::debugMode == 1) BeginTimer("ObtainSP_DAG");
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric) {
        std::cout << "####ObtainSP_DAG: ObtainSP_TaskSet ... "<<std::endl;
    }
#endif    
    double sp_overall = ObtainSP_TaskSet(dag_tasks.tasks, sp_parameters);
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric) {
        std::cout << "####ObtainSP_DAG: sp_overall = "<<sp_overall<<std::endl;
    }
#endif
    std::vector<FiniteDist> reaction_time_dists =
        GetRTDA_Dist_AllChains<ObjReactionTime>(dag_tasks);
    std::vector<double> chains_ddl = GetChainsDDL(dag_tasks);

#if defined(RYAN_HE_CHANGE_DEBUG)
    if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric) {
        if (reaction_time_dists.size()>0) {
            std::cout << "####ObtainSP_DAG: reaction_time_dists = ";
            for (int i = 0; i < reaction_time_dists.size(); i++) {
                std::cout << "reaction_time_dists["<<i << "]: "<<std::endl;
                reaction_time_dists[i].print();
            }
            std::cout << std::endl;
        }
        if (chains_ddl.size()>0) {
            std::cout << "####ObtainSP_DAG: chains_ddl = ";
            for (int i = 0; i < chains_ddl.size(); i++) {
                std::cout << chains_ddl[i] << " ";
            }
            std::cout << std::endl;
        }
    }
#endif
    for (int i = 0; i < reaction_time_dists.size(); i++) {
        int chain_id = i;
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric) {
            std::cout << "####ObtainSP_DAG: chain_id = "<<chain_id<<", GetDDL_MissProbability ... "<<std::endl;
        }
#endif
        double ddl_miss_chance =
            GetDDL_MissProbability(reaction_time_dists[i], chains_ddl[i]);
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (GlobalVariables::debugMode & DBG_PRT_MSK_SP_Metric) {
            std::cout << "####ObtainSP_DAG: chain_id = "<<chain_id<<", ddl_miss_chance = "<<ddl_miss_chance<<", SP_Func ... "<<std::endl;
        }
#endif            
        sp_overall += SP_Func(ddl_miss_chance,
                              sp_parameters.thresholds_path.at(chain_id)) *
                      sp_parameters.weights_path.at(chain_id);
    }

    if (GlobalVariables::debugMode == 1) EndTimer("ObtainSP_DAG");
    return sp_overall;
}

double ObtainSP_DAG_From_Dists(
    const DAG_Model& dag_tasks, const SP_Parameters& sp_parameters,
    const std::vector<FiniteDist>& node_rts_dists,
    const std::vector<FiniteDist>& path_latency_dists   
    #if defined(RYAN_HE_CHANGE)
    ,std::ofstream *log_stream
    #endif
    ) {      
    double sp_overall = 0;
    int print_weight = 0; // not to print to cout since python will parse std::cout
    if (print_weight)
        std::cerr << "weight: "; // << std::endl;
    for (uint i = 0; i < dag_tasks.tasks.size(); i++) {
        int task_id = dag_tasks.tasks[i].id;
        double sp_val = ObtainSP(node_rts_dists[i], dag_tasks.tasks[i].deadline,
                                 sp_parameters.thresholds_node.at(task_id),
                                 sp_parameters.weights_node.at(task_id));

        // std::cout << dag_tasks.tasks[i].name << " " << sp_val << std::endl;
#if defined(RYAN_HE_CHANGE)
        if (log_stream) {
            *log_stream << "task " << i << ": sp_val=" << sp_val << std::endl;
        }
#endif

        sp_overall += sp_val;

        if (print_weight)
            std::cerr << sp_parameters.weights_node.at(task_id) << " ";
    }
    if (print_weight)
        std::cerr << std::endl;

    for (uint i = 0; i < dag_tasks.chains_.size(); i++) {
        double v =
            ObtainSP(path_latency_dists[i], dag_tasks.chains_deadlines_[i],
                     sp_parameters.thresholds_path.at(i),
                     sp_parameters.weights_path.at(i));
        sp_overall += v;
        std::cerr << sp_parameters.weights_path.at(i) << " ";
    }

    return sp_overall;
}

double GetTaskPerfTerm(
    double ext_time_single,
    const std::vector<TimePerfPair>& timePerformancePairs_Sorted) {
    auto itr = std::lower_bound(
        timePerformancePairs_Sorted.begin(), timePerformancePairs_Sorted.end(),
        ext_time_single, [](const TimePerfPair& pair, double ext_time_single) {
            return pair.time_limit < ext_time_single;
        });
    if (itr == timePerformancePairs_Sorted.end()) {
        return timePerformancePairs_Sorted.back().performance;
    }
    if (itr == timePerformancePairs_Sorted.begin()) {  // should never happen
        return timePerformancePairs_Sorted.begin()->performance;
    }
    if (ext_time_single == itr->time_limit) return itr->performance;
    auto itr_prev = itr - 1;
    if (itr_prev->time_limit <= ext_time_single &&
        ext_time_single < itr->time_limit) {
        return itr_prev->performance;
    } else {
        CoutError(
            "Input time performance pairs are not sorted based on time!\n");
    }
    return 0;
}

double GetAvgTaskPerfTerm(std::string& ext_file_path,
                          std::vector<TimePerfPair> timePerformancePairs) {
    if (timePerformancePairs.size() == 0) {
        CoutError("timePerformancePairs is empty!");
        return 1.0;
    }
    std::vector<double> ext_times = ReadTxtFile(ext_file_path);
    int n = ext_times.size();
    if (n == 0) {
        // CoutWarning("ext_file_path is empty!");
        return 0.0;
    }
    double avg_perf_coeff = 0;
    for (int i = 0; i < n; i++) {
        avg_perf_coeff += GetTaskPerfTerm(ext_times[i], timePerformancePairs);
    }
    return avg_perf_coeff / n;
}

// RYAN_HE: added a dbg print flag for debugging purposes
// this is because this function can be called by python script
// dbg can be a cmdline arg (but by default should not use it)
double ObtainSPFromRTAFiles(std::string& slam_path, std::string& rrt_path,
                            std::string& mpc_path, std::string& tsp_path,
                            std::string& tsp_ext_path, std::string& chain0_path,
                            std::string& file_path_ref
#if defined(RYAN_HE_CHANGE)
                            , int dbg
#endif
                            ) {
    int granularity = GlobalVariables::Granularity;
    DAG_Model dag_tasks =
        ReadDAG_Tasks(file_path_ref);  // only read the tasks without worrying
                                       // about the execution time distribution

    SP_Parameters sp_parameters = ReadSP_Parameters(file_path_ref);
    assert(dag_tasks.tasks[0].name == "TSP");
    double tsp_weight = GetAvgTaskPerfTerm(
        tsp_ext_path, dag_tasks.tasks[0].timePerformancePairs);
    sp_parameters.update_node_weight(0, tsp_weight);
    std::vector<FiniteDist> node_rts_dists;

    // IN EACH FILE, every line is a double number in seconds
    // they are response time and execution (for TSP type tasks)
    // std::string folder_path="TaskData/AnalyzeSP_Metric/";
    node_rts_dists.push_back(FiniteDist(ReadTxtFile(tsp_path), granularity));
    node_rts_dists.push_back(FiniteDist(ReadTxtFile(mpc_path), granularity));
    node_rts_dists.push_back(FiniteDist(ReadTxtFile(rrt_path), granularity));
    node_rts_dists.push_back(FiniteDist(ReadTxtFile(slam_path), granularity));

#if defined(RYAN_HE_CHANGE)
    // RYAN_HE: chain is not always available so just allow it to be empty
    // NOTE: this file seems always assuming TSP/MPC/RRT/SLAM available
    // need to be more flexible (should read from task_characteristics.yaml?)
    std::vector<FiniteDist> reaction_time_dists;
    if (chain0_path != "") {
        if (dbg)
            std::cout << "####ObtainSPFromRTAFiles: chain0_path = " << chain0_path
                      << std::endl;        
        reaction_time_dists.push_back(FiniteDist(ReadTxtFile(chain0_path), granularity));
    } else {
        if (dbg)
            std::cout << "####ObtainSPFromRTAFiles: chain0_path empty " << std::endl;
    }
#else
    std::vector<FiniteDist> reaction_time_dists = {
        FiniteDist(ReadTxtFile(chain0_path), granularity)};
#endif
    if (dbg) {
        std::cout << "####ObtainSPFromRTAFiles: tsp_path = " << tsp_path
                << std::endl;
        std::cout << "####ObtainSPFromRTAFiles: mpc_path = " << mpc_path
                << std::endl;
        std::cout << "####ObtainSPFromRTAFiles: rrt_path = " << rrt_path
                << std::endl;
        std::cout << "####ObtainSPFromRTAFiles: slam_path = " << slam_path
                << std::endl;

        std::cout << "####ObtainSPFromRTAFiles: tsp_weight = " << tsp_weight
                << std::endl;
        std::cout << "####ObtainSPFromRTAFiles: reaction_time_dists (chain0) len = "
                << reaction_time_dists.size() << std::endl;

        for (int i = 0; i < node_rts_dists.size(); i++) {
            if (i==0)
                std::cout << "####ObtainSPFromRTAFiles: TSP distribution: " << std::endl;
            else if (i==1)
                std::cout << "####ObtainSPFromRTAFiles: MPC distribution: " << std::endl;
            else if (i==2)
                std::cout << "####ObtainSPFromRTAFiles: RRT distribution: " << std::endl;
            else if (i==3)
                std::cout << "####ObtainSPFromRTAFiles: SLAM distribution: " << std::endl;
            std::cout << "    len = " << node_rts_dists[i].distribution.size()
                    << std::endl;
            std::cout << "    min = " << node_rts_dists[i].min_time << std::endl;
            std::cout << "    max = " << node_rts_dists[i].max_time << std::endl;
            std::cout << "    avg = " << node_rts_dists[i].GetAvgValue()
                    << std::endl;
        }
    }
    double sp_value_overall = ObtainSP_DAG_From_Dists(
        dag_tasks, sp_parameters, node_rts_dists, reaction_time_dists);
    return sp_value_overall;
}

#if defined(RYAN_HE_CHANGE)
// using content in file_path_ref to
// read dag_tasks and sp_parameters
// get data files in the data_dir (TASKNAME_response_time.txt, TASKNAME_execution_time.txt)

//#define DBG_OUTPUT_FILE
// RYAN_HE_CHANGE_20250207
double ObtainSPFromRTAFiles2(std::string& file_path_ref, std::string& data_dir, int dbg) {
    int granularity = GlobalVariables::Granularity;

     // only read the tasks without worrying about the execution time distribution
    DAG_Model dag_tasks = ReadDAG_Tasks(file_path_ref); 

    // read sp parameters
    SP_Parameters sp_parameters = ReadSP_Parameters(file_path_ref);

    // if data_dir not ends with "/", add "/"
    if (data_dir.length() == 0) {
        data_dir = "./";
    } else if (data_dir[data_dir.length()-1] != '/') {
        data_dir = data_dir + "/";
    }

    #if defined(DBG_OUTPUT_FILE)
    std::ofstream *dbg_output_file = nullptr;
    if (1) {
        auto now = std::chrono::system_clock::now();
        // Convert to seconds since epoch
        auto epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
        // Get the long int timestamp
        long int timestamp = epoch.count();        
        std::ostringstream ss;
        ss << "SP_Metric_" << timestamp << ".txt";
        dbg_output_file = new std::ofstream(ss.str());
        if (!dbg_output_file->is_open()) {
            delete dbg_output_file;
            dbg_output_file = nullptr;
        }
    }
    #endif

    // get all task names and data files
    // IN EACH FILE, every line is a double number in seconds
    // they are response time and execution (for TSP type tasks)    
    std::vector<FiniteDist> node_rts_dists;
    for (int i = 0; i < dag_tasks.tasks.size(); i++) {
        std::string rst_path = data_dir + dag_tasks.tasks[i].name + "_response_time.txt";
        if (dbg) {
            std::cout << "####ObtainSPFromRTAFiles2: rst_path = " << rst_path << std::endl;
        }
        node_rts_dists.push_back(FiniteDist(ReadTxtFile(rst_path), granularity));

        if ( dag_tasks.tasks[i].timePerformancePairs.size() > 0 ) {
            std::string ext_path = data_dir + dag_tasks.tasks[i].name + "_execution_time.txt";
            double weight = GetAvgTaskPerfTerm(ext_path, dag_tasks.tasks[i].timePerformancePairs);

            #if defined(DBG_OUTPUT_FILE)
            if (dbg_output_file) {
                (*dbg_output_file) << "with perf, task:" << i<< ", weight=" << weight << std::endl;
            }
            #endif

            sp_parameters.update_node_weight(i, weight);
            if (dbg) {
                std::cout << "####ObtainSPFromRTAFiles2: ext_path = " << ext_path << ", weight = " << weight << std::endl;
            }
        }
    }


    // RYAN_HE: chain is not always available so just allow it to be empty
    std::vector<FiniteDist> reaction_time_dists;
    // RYAN_HE_CHANGE_20250207
    double sp_value_overall = ObtainSP_DAG_From_Dists(dag_tasks, sp_parameters, 
                                                      node_rts_dists, reaction_time_dists
                                                      #if defined(RYAN_HE_CHANGE) && defined(DBG_OUTPUT_FILE)
                                                      , dbg_output_file
                                                      #endif
                                                      );

    #if defined(DBG_OUTPUT_FILE)
    if (dbg_output_file) {
        (*dbg_output_file) << "sp_value_overall = " << sp_value_overall << std::endl;
        dbg_output_file->close();
        delete dbg_output_file;
        dbg_output_file = nullptr;
    }
    #endif

    return sp_value_overall;
}
#endif


}  // namespace SP_OPT_PA