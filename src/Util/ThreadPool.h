/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_THREAD_POOL_H
#define CNOID_UTIL_THREAD_POOL_H

#include <queue>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace cnoid {

class ThreadPool
{
private:
    std::vector<std::thread> threads;
    std::queue<std::function<void()>> queue;
    std::mutex mutex;
    std::condition_variable condition;
    std::condition_variable finishCondition;
    int numActiveThreads;
    bool isDestroying;
        
public:
    ThreadPool(int size = 1) {
        numActiveThreads = 0;
        isDestroying = false;
        for(int i = 0; i < size; ++i){
            threads.emplace_back(std::bind(&ThreadPool::run, this));
        }
    }
    
    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> guard(mutex);
            isDestroying = true;
            condition.notify_all();
        }
        // join_all
        for(auto& thread : threads){
            if(thread.joinable()){
                thread.join();
            }
        }
    }
    
    int size() const { return threads.size(); }
    
    void start(std::function<void()> f) {
        std::lock_guard<std::mutex> guard(mutex);
        queue.push(f);
        condition.notify_one();
    }
    
    void wait(){
        std::unique_lock<std::mutex> lock(mutex);
        while(!queue.empty() || numActiveThreads > 0){
            finishCondition.wait(lock);
        }
    }
    
    void waitLoop(){
        while(true){
            {
                std::unique_lock<std::mutex> lock(mutex, std::try_to_lock_t());
                if(lock.owns_lock()){
                    if(queue.empty() && numActiveThreads == 0){
                        break;
                    }
                }
            }
        }
    }

    bool isRunning() {
        std::lock_guard<std::mutex> guard(mutex);
        return numActiveThreads > 0;
    }
    
private:
    void run() {
        while(true){
            std::function<void()> f;
            {
                std::unique_lock<std::mutex> lock(mutex);
                
                while (queue.empty() && !isDestroying){
                    condition.wait(lock);
                }
                if(!queue.empty()){
                    f = queue.front();
                    queue.pop();
                    ++numActiveThreads;
                }
            }
            if(f){
                f();
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    --numActiveThreads;
                    if(numActiveThreads == 0){
                        finishCondition.notify_all();
                    }
                }
            } else {
                break;
            }
        }
    }
};

}
    
#endif
