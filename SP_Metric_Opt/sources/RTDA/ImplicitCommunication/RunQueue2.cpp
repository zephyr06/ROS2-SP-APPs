#include "sources/RTDA/ImplicitCommunication/RunQueue2.h"


//namespace DAG_SPACE {
namespace SP_OPT_PA {

void RunQueue2::insert(JobCEC job, LLint time_now) {
    // RYAN_HE: for RunQueue2, need to set deadline when creating job_info
    LLint new_d = tasks_info_.GetTask(job.taskId).deadline + time_now;
    JobScheduleInfo job_info(job,new_d);
#if defined(RYAN_HE_CHANGE_DEBUG)
    if (dbg){
      double priorityCurr = tasks_info_.GetTask(job.taskId).get_priority2(time_now,new_d,job.jobId);		
      std::cout<<"####RunQueue2 insert: t="<<time_now<<", task"<<job.taskId<<", deadlineJob="<<new_d<<", prio="<<priorityCurr<<std::endl;
    }
#endif
	
    if (job_queue_.size() == 0) {
#if defined(RYAN_HE_CHANGE_DEBUG)
      if (dbg){
        std::cout<<"####RunQueue2 insert: 1 job, taskId="<<job_info.job.taskId<<", jobId="<<job_info.job.jobId<<std::endl;
      }
#endif        
      job_queue_.push_back(job_info);
    } else {
      LLint prev = FindPrev(job.taskId,job.jobId,time_now,new_d);	
 
#if defined(RYAN_HE_CHANGE_DEBUG)
      if (dbg) {
        std::cout<<"####RunQueue2 insert: after pos="<<prev<<", taskId="<<job_info.job.taskId<<", jobId="<<job_info.job.jobId<<std::endl;
      }
#endif          
      job_queue_.insert(job_queue_.begin() + prev, job_info);
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
// RYAN_HE: for RunQueue2, time_now is needed to compute priority when calling get_priority2
// for EDF, priority depends on time_now
LLint RunQueue2::FindPrev(LLint task_id, LLint job_id, LLint time_now, LLint deadlineJob) {
    Task taskCurr = tasks_info_.GetTask(task_id);
	
	// priority computation should also include deadline etc.
    double priorityCurr = taskCurr.get_priority2(time_now,deadlineJob,job_id);

#if defined(RYAN_HE_CHANGE_DEBUG)
    if (dbg){
      std::cout<<"####RunQueue2 FindPrev: taskCurr.id="<<taskCurr.id<<", deadlineJob="<<deadlineJob<<", prio="<<priorityCurr<<std::endl;
    }
#endif 
	
    LLint left = 0;
    LLint right = job_queue_.size();
    while (left < right) {
      LLint mid = (left + right) / 2;
      // double priorityMid = tasks[job_queue_[mid].job.taskId].priority();
	  LLint deadlineJob = job_queue_[mid].deadlineJob;
      double priorityMid = tasks_info_.GetTask(job_queue_[mid].job.taskId).get_priority2(time_now,deadlineJob,job_queue_[mid].job.jobId); 
#if defined(RYAN_HE_CHANGE_DEBUG)
      if (dbg){
        std::cout<<"####RunQueue2 FindPrev: left="<<left<<", right="<<right<<", mid="<<mid<<" taskMid.id="<<job_queue_[mid].job.taskId<<", prio="<<priorityMid<<std::endl;
      }
#endif 	  
      if (priorityMid == priorityCurr) {
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (dbg){
          std::cout<<"####RunQueue2 FindPrev: prio curr==mid, return mid pos="<<mid<<std::endl;
        }
#endif 			  
        return mid;
      } else if (priorityCurr < priorityMid) {
        // 假设priority是从大到小排列的			
        left = mid + 1;
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (dbg){
          std::cout<<"####RunQueue2 FindPrev: prio curr > mid, left pos="<<left<<std::endl;
        }
#endif 		
      } else {
        right = mid;
#if defined(RYAN_HE_CHANGE_DEBUG)
        if (dbg){
          std::cout<<"####RunQueue2 FindPrev: prio curr < mid, right pos="<<right<<std::endl;
        }
#endif 		
      }
    }

#if defined(RYAN_HE_CHANGE_DEBUG)
    if (dbg){
        std::cout<<"####RunQueue2 FindPrev: finally return left, left="<<left<<", right="<<right<<std::endl;
    }
#endif 	
    return left;
}

void RunQueue2::print_jobs_in_queue(int time_now) {
  std::cout<<"####RunQueue2::print_jobs_in_queue ..."<<std::endl; 
  int sz = job_queue_.size();
  for (int i=0;i<sz;i++) {
    int taskId = job_queue_[i].job.taskId;
	  int jobId = job_queue_[i].job.jobId;	  
	  LLint deadlineJob = job_queue_[i].deadlineJob;
	  double prio = tasks_info_.GetTask(taskId).get_priority2(time_now,deadlineJob,jobId);
    std::cout<<"  taskId="<<taskId<<", jobId="<<jobId<<", deadlineJob="<<deadlineJob<<", prio="<<prio<<std::endl; 
  }
  std::cout<<std::endl;  	
}

// not call
void RunQueue2::update_deadlineJob_in_queue() {
  int sz = job_queue_.size();
  for (int i=0;i<sz;i++) {
    job_queue_[i].deadlineJob -= 1;
  }
}


}