#pragma once



#include <memory>
#include <functional>
#include <thread>
#include <vector>

#include <boost/thread/latch.hpp>
#include <boost/iterator/iterator_facade.hpp>


#include "tbb/concurrent_queue.h"




template <class T>
class BlockRange
{
public:

	BlockRange( T		begin,
				T		end)
		: m_begin(begin),
		  m_end(end)
	{}


	T			begin() const
	{
		return(m_begin);
	}

	T			end() const
	{
		return(m_end);
	}


private:

	T			m_begin;
	T			m_end;
};




class ITask
{
public :

	virtual void		Execute(void) = 0;
};



template <typename T>
class ParallelForBlock : public ITask
{
public :

	typedef std::function<void(T, T)>		ExecutableBody;

	ParallelForBlock( boost::latch&			latch,
	                  T						itrBegin,
					  T						itrEnd,
					  ExecutableBody		executable )
		: m_executable(executable),
		  m_itrBegin( itrBegin ),
		  m_itrEnd( itrEnd ),
		  m_latch( latch )
	{}


	void		Execute(void)
	{
		m_executable( m_itrBegin, m_itrEnd );

		m_latch.count_down();
	}


private :

	ExecutableBody				m_executable;

	T		m_itrBegin;
	T		m_itrEnd;

	boost::latch&				m_latch;
};



template <typename T>
class ParallelForBlockRanged : public ITask
{
public:

	typedef std::function<void(BlockRange<T>)>		ExecutableBody;

	ParallelForBlockRanged( boost::latch&		latch,
							T			itrBegin,
							T			itrEnd,
							ExecutableBody		executable)
		: m_executable(executable),
		  m_itrBegin(itrBegin),
		  m_itrEnd(itrEnd),
		  m_latch(latch)
	{}


	void		Execute(void)
	{
		m_executable( BlockRange<T>( m_itrBegin, m_itrEnd ));

		m_latch.count_down();
	}


private:

	ExecutableBody			m_executable;

	T				m_itrBegin;
	T				m_itrEnd;

	boost::latch&			m_latch;
};





class ThreadPool
{
public :

	ThreadPool()
		: m_numCores(std::thread::hardware_concurrency() / 2)
	{
		for (int i = 0; i < m_numCores; i++)
		{
			m_workers.emplace_back(&ThreadPool::ThreadMain, this);
		}
	}


	static ThreadPool&				getPool();



	void		ScheduleTask(std::shared_ptr<ITask>		task)
	{
		m_taskQueue.push(task);
	}



	template <typename T>
	void	parallel_for( int	numTasks, T begin, T end, typename ParallelForBlock<T>::ExecutableBody  body )
	{
		if(( end - begin ) == 0)
		{
			return;
		}

		if (numTasks < 1)
		{
			numTasks = 1;
		}

		numTasks = std::min(numTasks, m_numCores);
		numTasks = std::min((long)numTasks, (long)(end-begin));

		boost::latch		completionLatch(numTasks - 1);

		size_t				blockSize = (end - begin) / numTasks;

		T					currentBegin = begin;
		
		for (int i = 0; i < numTasks - 1; i++)
		{
			ScheduleTask(std::shared_ptr<ITask>(new ParallelForBlock<T>(completionLatch, currentBegin, currentBegin + blockSize, body)));

			currentBegin += blockSize;
		}

		body(currentBegin, end );

		completionLatch.wait();
	}


	template <typename T>
	void	parallel_for(int	numTasks, T begin, T end, typename ParallelForBlockRanged< T>::ExecutableBody  body)
	{
		if ((end - begin) == 0)
		{
			return;
		}

		if (numTasks < 1)
		{
			numTasks = 1;
		}

		numTasks = std::min(numTasks, m_numCores);
		numTasks = std::min((long)numTasks, (long)(end - begin));

		boost::latch		completionLatch(numTasks - 1);

		size_t				blockSize = (end - begin) / numTasks;

		T					currentBegin = begin;

		for (int i = 0; i < numTasks - 1; i++)
		{
			ScheduleTask(std::shared_ptr<ITask>(new ParallelForBlockRanged<T>(completionLatch, currentBegin, currentBegin + blockSize, body)));

			currentBegin += blockSize;
		}

		body( BlockRange<T>( currentBegin, end ));

		completionLatch.wait();
	}


private :

	typedef tbb::concurrent_bounded_queue<std::shared_ptr<ITask>>		TaskQueue;


	const int						m_numCores;

	TaskQueue						m_taskQueue;

	std::vector<std::thread>		m_workers;



	void		ThreadMain()
	{
		while (true)
		{
			std::shared_ptr<ITask>		nextTask;
			
			m_taskQueue.pop( nextTask );

			nextTask->Execute();
		}

	}

};






