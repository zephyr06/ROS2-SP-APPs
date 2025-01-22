#pragma once
#include "sources/RTDA/JobScheduleInfo.h"
#include "sources/Utils/Parameters.h"

#include <fstream> 

//namespace DAG_SPACE {
namespace SP_OPT_PA {

class RunQueueCSP {
 public:
  RunQueueCSP(
    const TaskSetInfoDerived tasks_info
  ) : tasks_info_(tasks_info) {
    N = tasks_info.N;
    job_queue_.reserve(N);
    schedule_.reserve(tasks_info.length);
    last_task_ = -1;
  }

  inline size_t size() const { return job_queue_.size(); }

  inline Schedule GetSchedule() const { return schedule_; }

  inline void RangeCheck(size_t job_info_index_in_queue) {
    if (job_info_index_in_queue < 0 ||
        job_info_index_in_queue >= job_queue_.size())
      CoutError("Index out of range in RunQueueCSP");
  }

  // just put to end
  void insert(JobCEC job, LLint time_now, int executionTime);

  int is_task_running(int task_id);

  // exam whether the running job has highest priority, if not, preempt it
  void RunJobHigestPriority(LLint time_now);

  // put current job to the end and run the next in queue
  void RotateRunNextJob(LLint time_now);

  void PreemptJob(LLint time_now) {
    // TODO: improve efficiency
    for (uint i = 0; i < size(); i++) {
      PreemptJob(i, time_now);
    }
  }

  void PreemptJob(size_t job_info_index_in_queue, LLint time_now) {
    RangeCheck(job_info_index_in_queue);
    JobScheduleInfo &job_info = job_queue_[job_info_index_in_queue];
    if (job_info.running == false)
      return;
    else {
      job_info.UpdateAccumTime(time_now);
      job_info.running = false;
      processor_free_ = true;
    }
  }

  bool RunJob(size_t job_info_index_in_queue, LLint time_now) {
    RangeCheck(job_info_index_in_queue);
    if (!processor_free_) return false;
    processor_free_ = false;
    JobScheduleInfo &job_info = job_queue_[job_info_index_in_queue];
    if (dbg){
      std::cout<<"####RunQueueCSP RunJob: run job pos="<<job_info_index_in_queue<<", time_now="<<time_now;
      std::cout<<", taskId="<<job_info.job.taskId<<", jobId="<<job_info.job.jobId<<std::endl;
    }	
    job_info.StartRun(time_now);

    auto itr = schedule_.find(job_info.job);
    if (itr == schedule_.end()) {  // record the start time only when first accessing it
      JobStartFinish job_sf(time_now, -1, job_info.executionTime); 
      schedule_[job_info.job] = job_sf;
      //schedule_[job_info.job].executionTime = job_info.executionTime;
    }

    next_free_time_ = time_now + job_info.executionTime - job_info.accum_run_time;
    return true;
  }

  void RemoveFinishedJob(LLint time_now, std::ofstream *output_file=NULL);

  inline JobScheduleInfo front() const {
    if (job_queue_.empty()) CoutError("job_queue_ is empty!");

    return job_queue_[0];
  }

  inline bool empty() { return job_queue_.empty(); }
  
  void print_jobs_in_queue(int time_now);
  
  void set_task_priorityType(int task_id, std::string priorityType);
  void set_task_priority(int task_id, double priority);
  void set_job_executionTime(int task_id, int executionTime);

  // data members

  // typedef std::pair<int, LLint> ID_INSTANCE_PAIR;  // taskId, jobId
  TaskSetInfoDerived tasks_info_;
  int N;
  std::vector<JobScheduleInfo> job_queue_;
  int next_free_time_ = 0;
  bool processor_free_ = true;
  Schedule schedule_;
  int last_task_ = -1;
  int dbg = 0;
};

}  // namespace DAG_SPACE