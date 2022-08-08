// Copyright (c) 2021 Stephan Friedl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "thread_pool.hpp"

#include <iostream>

namespace SEFUtility::threading
{
    std::unique_ptr<ThreadPool> g_threadPool;  //  NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

    ThreadPool& ThreadPool::getPool()
    {
        if (!g_threadPool)
        {
            g_threadPool = std::make_unique<ThreadPool>();
        }

        return (*g_threadPool);
    }

    namespace hidden
    {
        class NoOpTaskForShutdown : public ITask
        {
           public:
           NoOpTaskForShutdown() = default;
           NoOpTaskForShutdown(const NoOpTaskForShutdown&) = default;
           NoOpTaskForShutdown(NoOpTaskForShutdown&&) = default;

            virtual ~NoOpTaskForShutdown() = default;

            NoOpTaskForShutdown& operator=(const NoOpTaskForShutdown&) = delete;
            NoOpTaskForShutdown& operator=(NoOpTaskForShutdown&&) = delete;

            void Execute() override {}
        };
    }  //  namespace hidden

    void ThreadPool::Shutdown()
    {
        using namespace std::chrono_literals;

        running_ = false;

        //  Insert enough noop tasks to permit all workers to exit.

        //      Grab the number of workers in advance, as we don't want to slap a mutex
        //      block here to prevent concurrency issues with the workers_ vector.

        size_t num_workers = workers_.size();

        for (size_t i = 0; i < num_workers; i++)
        {
            std::shared_ptr<ITask> no_op_task(new hidden::NoOpTaskForShutdown());

            task_queue_.push(no_op_task);
        }

        //  Sleep for a second to permit enough time for all the workes to actually exit.

        std::this_thread::sleep_for(1s);

        for (auto& exited_worker : workers_exited_)
        {
            if (exited_worker.joinable())
            {
                exited_worker.join();
            }
        }

        if (!workers_.empty())
        {
            std::cout << "One or more worker threads are still running." << std::endl;
        }
    }

    void ThreadPool::ThreadMain()
    {
        while (running_)
        {
            std::shared_ptr<ITask> nextTask;

            task_queue_.pop(nextTask);

            nextTask->Execute();
        }

        //  We are exiting, so drain the trhead from the workers vector and add it to the workers exited vector.
        //      Both vectors are not thread-safe, so use a mutex here.

        {
            std::lock_guard<std::mutex> workers_lock_guard(workers_vector_mutex_);

            for (auto worker_itr = workers_.begin(); worker_itr != workers_.end(); worker_itr++)
            {
                if (worker_itr->get_id() == std::this_thread::get_id())
                {
                    workers_exited_.emplace_back(std::move(*worker_itr));

                    workers_.erase(worker_itr);

                    break;
                }
            }
        }
    }
}  // namespace SEFUtility::threading
