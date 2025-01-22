#include "sources/RTDA/ImplicitCommunication/RunQueueCSP.h"

//namespace DAG_SPACE {
namespace SP_OPT_PA {

void RunQueueCSP::insert(JobCEC job, LLint time_now, int executionTime) {
    // set deadline ...
    LLint new_d = tasks_info_.GetTask(job.taskId).deadline + time_now;
    if (dbg){
      std::cout<<"####RunQueueCSP::insert task"<<job.taskId<<" jobId"<<job.jobId<<", deadlineJob="<<new_d<<std::endl; 
    }
    JobScheduleInfo job_info(job,new_d,executionTime);
    job_queue_.push_back(job_info);
}

int RunQueueCSP::is_task_running(int task_id) {
  for (size_t i = 0; i < job_queue_.size(); i++) {
    if (job_queue_[i].job.taskId == task_id) {
      return 1;
    }
  }
  return 0;
}

void RunQueueCSP::set_job_executionTime(int task_id, int executionTime) {
  for (size_t i = 0; i < job_queue_.size(); i++) {
    if (job_queue_[i].job.taskId == task_id) {
      job_queue_[i].executionTime = executionTime;
      return;
    }
  }
  // CoutError("job not found");
}

void RunQueueCSP::print_jobs_in_queue(int time_now) {
  std::cout<<"####RunQueueCSP::print_jobs_in_queue ..."<<std::endl; 
  int sz = job_queue_.size();
  for (int i=0;i<sz;i++) {
    int taskId = job_queue_[i].job.taskId;
  	int jobId = job_queue_[i].job.jobId;	  
  	LLint deadlineJob = job_queue_[i].deadlineJob;
    std::string priorityType = tasks_info_.GetTask(taskId).priorityType_;
    double prio = tasks_info_.GetTask(taskId).get_priority2(time_now,deadlineJob,jobId);
    std::cout<<"  taskId="<<taskId<<", jobId="<<jobId<<", deadlineJob="<<deadlineJob;
    std::cout<<", priorityType="<<priorityType<<", prio="<<prio<<", priorityType="<<priorityType<<std::endl; 
  }
  std::cout<<std::endl;  
}

void RunQueueCSP::set_task_priorityType(int taskId, std::string priorityType) {
  tasks_info_.GetTaskForPriority(taskId).priorityType_ = priorityType;
}

void RunQueueCSP::set_task_priority(int task_id, double priority) {
  tasks_info_.GetTaskForPriority(task_id).set_priority(priority);
}

void RunQueueCSP::RemoveFinishedJob(LLint time_now, std::ofstream *output_file) {
    for (size_t i = 0; i < job_queue_.size(); i++) {
      auto &job_info = job_queue_[i];
      if (job_info.running) {
        job_info.UpdateAccumTime(time_now);
      }
      if (job_info.IfFinished()) {
        if (job_info.running) {
          job_info.running = false;
          schedule_[job_info.job].finish = time_now;
          processor_free_ = true;
          int taskId = job_info.job.taskId;
          int jobId = job_info.job.jobId;
          int start = schedule_[job_info.job].start;
          int finish = schedule_[job_info.job].finish;
          int exe_time = job_info.executionTime;
          if (output_file) {
            *output_file << taskId << "," << jobId << "," << start << "," << finish << "," << exe_time << std::endl;
          }
          if (dbg) {
            // schedule_[job_info.job].start,finish,
            std::cout<<"####RunQueueCSP::RemoveFinishedJob: time_now="<<time_now;
            std::cout<<", ("<<taskId<<", "<<jobId<<"), ";
            std::cout<<"start/finish/exe_time="<<start<<"/"<<finish<<"/"<<exe_time<<std::endl;
          }
          last_task_ = -1;
        }
        job_queue_.erase(job_queue_.begin() + i);
      }
    }
}
  
// exam whether the running job has highest priority, if not, preempt it
void RunQueueCSP::RunJobHigestPriority(LLint time_now) {
  if (empty()) {
    return;
  }
  else if (size() == 1) {
    if (dbg){
      std::cout<<"####RunQueueCSP RunJobHigestPriority: time_now="<<time_now<<", 1 job"<<std::endl;
    }
    RunJob(0, time_now);
    last_task_ = job_queue_[0].job.taskId;
  } else {		
      
    // TRY TO FIND BEST PROIORITY
    int sz = job_queue_.size();
    int best_task = -1;
    double best_prio = -1;
    int best_idx = -1;
    for (int i=0;i<sz;i++) {
      int taskId = job_queue_[i].job.taskId;
      int jobId = job_queue_[i].job.jobId;	  
      double prio = tasks_info_.GetTask(taskId).get_priority();
      if (dbg)
        std::cout<<"####RunQueueCSP::RunJobHigestPriority: taskId="<<taskId<<", jobId="<<jobId<<", prio="<<prio<<std::endl; 
      if (best_task == -1 || prio > best_prio) {
        best_task = taskId;
        best_prio = prio;
        best_idx = i;
      }
    }

    if (dbg){
      std::cout<<"####RunQueueCSP::RunJobHigestPriority: time_now="<<time_now<<", "<<size()<<" jobs";
    }
    if (best_task == last_task_) {
      auto job_info = job_queue_[best_idx];
      if (job_info.running) {
        if (dbg){
          std::cout<<", still run curr, taskId="<<job_info.job.taskId<<", jobId="<<job_info.job.jobId<<std::endl;
        }
      }	else {
        if (dbg){
          std::cout<<", SHOULD run curr, taskId="<<job_info.job.taskId<<", jobId="<<job_info.job.jobId<<std::endl;      
          CoutError("WHY IT WAS NOT RUNNING?");    
        }
      }
      return;
    }
    else {
      if (dbg){
        std::cout<<", preempt curr and run task "<<best_task<<std::endl;
      }			  
      PreemptJob(time_now);  // preempty running jobs, if any
      if (!RunJob(best_idx, time_now)) {
        CoutError("Error in RunQueueCSP RunJobHigestPriority!");
      }
      last_task_ = best_task;
    }
  }
}

void RunQueueCSP::RotateRunNextJob(LLint time_now) {
  if (empty()) {
    return;
  }
  else if (size() == 1) {
    if (dbg){
      std::cout<<"####RunQueueCSP RotateRunNextJob: time_now="<<time_now<<", 1 job"<<std::endl;
    }
    RunJob(0, time_now);
    last_task_ = job_queue_[0].job.taskId;
  } else {		

    PreemptJob(time_now);  // preempty running jobs (the first)

    JobScheduleInfo first_job = job_queue_.front();  // Get the first element
    
    job_queue_.erase(job_queue_.begin());            // Remove the first element
    job_queue_.push_back(first_job);      

    int old_task_id = first_job.job.taskId;
    int old_job_id = first_job.job.jobId;
    int new_task_id = job_queue_[0].job.taskId;
    int new_job_id = job_queue_[0].job.jobId;
    if (dbg){
      std::cout<<"####RunQueueCSP::RotateRunNextJob: time_now="<<time_now<<", "<<size()<<" jobs"<<std::endl;
      std::cout<<"     old_task/job_id="<<old_task_id<<"/"<<old_job_id;
      std::cout<<", new_task/job_id="<<new_task_id<<"/"<<new_job_id<<std::endl;
    }
    if (!RunJob(0, time_now)) {
      CoutError("Error in RunQueueCSP RunJobHigestPriority!");
    }
    last_task_ = 0;
  }
}

} // namespace SP_OPT_PA {