#pragma once
#include "sources/RTDA/JobScheduleInfo.h"

//namespace DAG_SPACE {
namespace SP_OPT_PA {

class RunQueue2 {
 public:
  RunQueue2(
    const TaskSetInfoDerived tasks_info
  ) : tasks_info_(tasks_info) {
    N = tasks_info.N;
    job_queue_.reserve(N);
    schedule_.reserve(tasks_info.length);
  }

  inline size_t size() const { return job_queue_.size(); }

  inline Schedule GetSchedule() const { return schedule_; }

  inline void RangeCheck(size_t job_info_index_in_queue) {
    if (job_info_index_in_queue < 0 ||
        job_info_index_in_queue >= job_queue_.size())
      CoutError("Index out of range in RunQueue2");
  }

  void insert(JobCEC job, LLint time_now);

  // exam whether the running job has highest priority, if not, preempt it
  void RunJobHigestPriority(LLint time_now) {
    if (empty())
      return;
    else if (size() == 1) {
      if (dbg){
        std::cout<<"####RunQueue2 RunJobHigestPriority: time_now="<<time_now<<", 1 job"<<std::endl;
      }
      RunJob(0, time_now);
    } else {		
      auto job_info = front();
      if (job_info.running) {
        if (dbg){
          std::cout<<"####RunQueue2 RunJobHigestPriority: time_now="<<time_now<<", "<<size()<<" jobs, still run curr, taskId="<<job_info.job.taskId<<", jobId="<<job_info.job.jobId<<std::endl;
        }	  
        return;
	    }
      else {
        if (dbg){
          std::cout<<"####RunQueue2 RunJobHigestPriority: time_now="<<time_now<<", "<<size()<<" jobs, preempt curr and run new"<<std::endl;
        }			  
        PreemptJob(time_now);  // preempty running jobs, if any
        if (!RunJob(0, time_now)) {
          CoutError("Error in RunQueue2 RunJobHigestPriority!");
		    }
      }
    }
  }

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
      std::cout<<"####RunQueue2 RunJob: run job pos="<<job_info_index_in_queue<<", time_now="<<time_now<<", taskId="<<job_info.job.taskId<<", jobId="<<job_info.job.jobId<<std::endl;
    }	
    job_info.StartRun(time_now);

    auto itr = schedule_.find(job_info.job);
    if (itr == schedule_.end()) {  // record the start time only when first accessing it
      JobStartFinish job_sf(time_now, -1);
      schedule_[job_info.job] = job_sf;
    }

    next_free_time_ = time_now + job_info.executionTime - job_info.accum_run_time;
    return true;
  }

  void RemoveFinishedJob(LLint time_now) {
    for (size_t i = 0; i < job_queue_.size(); i++) {
      auto &job_info = job_queue_[i];
      if (job_info.running) {
        job_info.UpdateAccumTime(time_now);
      }
      if (job_info.IfFinished(tasks_info_)) {
        if (job_info.running) {
          job_info.running = false;
          schedule_[job_info.job].finish = time_now;
          processor_free_ = true;
        }
        job_queue_.erase(job_queue_.begin() + i);
      }
    }
  }

  /**
   * @brief Given a new task's id, find its position in the queue
   * such that the queue is ordered from highest priority (largest numbers in
   * priority()) to lowest priority
   *
   * @param id: new task's id
   * @return LLint
   */
  LLint FindPrev(LLint task_id, LLint job_id, LLint time_now, LLint deadlineJob);

  inline JobScheduleInfo front() const {
    if (job_queue_.empty()) CoutError("job_queue_ is empty!");

    return job_queue_[0];
  }

  inline bool empty() { return job_queue_.empty(); }

  
  void print_jobs_in_queue(int time_now);
  void update_deadlineJob_in_queue();
  
  // data members

  // typedef std::pair<int, LLint> ID_INSTANCE_PAIR;  // taskId, jobId
  TaskSetInfoDerived tasks_info_;
  int N;
  std::vector<JobScheduleInfo> job_queue_;
  int next_free_time_ = 0;
  bool processor_free_ = true;
  Schedule schedule_;
  int dbg = 0;
};

}  // namespace DAG_SPACE