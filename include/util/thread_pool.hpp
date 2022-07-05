#pragma once

#include <boost/iterator/iterator_facade.hpp>
#include <boost/thread/latch.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "oneapi/tbb/concurrent_queue.h"

template <class T>
class BlockRange
{
   public:
    BlockRange(T begin, T end) : begin_(begin), end_(end) {}

    T begin() const { return (begin_); }

    T end() const { return (end_); }

   private:
    T begin_;
    T end_;
};

class ITask
{
   public:
    virtual void Execute(void) = 0;
};

template <typename T>
class ParallelForBlock : public ITask
{
   public:
    typedef std::function<void(T, T)> ExecutableBody;

    ParallelForBlock(boost::latch& latch, T itrBegin, T itrEnd, ExecutableBody executable)
        : m_executable(executable), itr_begin_(itrBegin), itr_end_(itrEnd), latch_(latch)
    {
    }

    void Execute(void)
    {
        m_executable(itr_begin_, itr_end_);

        latch_.count_down();
    }

   private:
    ExecutableBody m_executable;

    T itr_begin_;
    T itr_end_;

    boost::latch& latch_;
};

template <typename T>
class ParallelForBlockRanged : public ITask
{
   public:
    typedef std::function<void(BlockRange<T>)> ExecutableBody;

    ParallelForBlockRanged(boost::latch& latch, T itrBegin, T itrEnd, ExecutableBody executable)
        : executable_(executable), itr_begin_(itrBegin), itr_end_(itrEnd), latch_(latch)
    {
    }

    void Execute(void)
    {
        executable_(BlockRange<T>(itr_begin_, itr_end_));

        latch_.count_down();
    }

   private:
    ExecutableBody executable_;

    T itr_begin_;
    T itr_end_;

    boost::latch& latch_;
};

class ThreadPool
{
   public:
    ThreadPool() : running_(true), num_cores_(std::thread::hardware_concurrency() / 2)
    {
        for (int i = 0; i < num_cores_; i++)
        {
            workers_.emplace_back(&ThreadPool::ThreadMain, this);
        }
    }

    static ThreadPool& getPool();

    void Shutdown()
    {
        using namespace std::chrono_literals;

        running_ = false;

        while (!workers_.empty())
        {
            std::shared_ptr<ITask> no_op_task(new NoOpTaskForShutdown());

            task_queue_.push(no_op_task);

            std::this_thread::sleep_for(1s);

            bool thread_joined = false;

            do
            {
				thread_joined = false;

                for (auto itr_thread = workers_.begin(); itr_thread != workers_.end(); itr_thread++)
                {
                    if (itr_thread->joinable())
                    {
                        itr_thread->join();
                        workers_.erase(itr_thread);

						thread_joined = true;

						break;
                    }
                }
            } while ( thread_joined );
        }
    }

    void ScheduleTask(std::shared_ptr<ITask> task)
    {
        if (!running_)
        {
            return;
        }

        task_queue_.push(task);
    }

    template <typename T>
    void parallel_for(int numTasks, T begin, T end, typename ParallelForBlock<T>::ExecutableBody body)
    {
        if ((end - begin) == 0)
        {
            return;
        }

        if (numTasks < 1)
        {
            numTasks = 1;
        }

        numTasks = std::min(numTasks, num_cores_);
        numTasks = std::min((long)numTasks, (long)(end - begin));

        boost::latch completionLatch(numTasks - 1);

        size_t blockSize = (end - begin) / numTasks;

        T currentBegin = begin;

        for (int i = 0; i < numTasks - 1; i++)
        {
            ScheduleTask(std::shared_ptr<ITask>(
                new ParallelForBlock<T>(completionLatch, currentBegin, currentBegin + blockSize, body)));

            currentBegin += blockSize;
        }

        body(currentBegin, end);

        completionLatch.wait();
    }

    template <typename T>
    void parallel_for(int numTasks, T begin, T end, typename ParallelForBlockRanged<T>::ExecutableBody body)
    {
        if ((end - begin) == 0)
        {
            return;
        }

        if (numTasks < 1)
        {
            numTasks = 1;
        }

        numTasks = std::min(numTasks, num_cores_);
        numTasks = std::min((long)numTasks, (long)(end - begin));

        boost::latch completionLatch(numTasks - 1);

        size_t blockSize = (end - begin) / numTasks;

        T currentBegin = begin;

        for (int i = 0; i < numTasks - 1; i++)
        {
            ScheduleTask(std::shared_ptr<ITask>(
                new ParallelForBlockRanged<T>(completionLatch, currentBegin, currentBegin + blockSize, body)));

            currentBegin += blockSize;
        }

        body(BlockRange<T>(currentBegin, end));

        completionLatch.wait();
    }

   private:
    using TaskQueue = tbb::concurrent_bounded_queue<std::shared_ptr<ITask>>;

    const int num_cores_;

    TaskQueue task_queue_;

    std::vector<std::thread> workers_;

    std::atomic_bool running_;

    class NoOpTaskForShutdown : public ITask
    {
       public:
        void Execute() {}
    };

    void ThreadMain()
    {
        while (running_)
        {
            std::shared_ptr<ITask> nextTask;

            task_queue_.pop(nextTask);

            nextTask->Execute();
        }
    }
};
