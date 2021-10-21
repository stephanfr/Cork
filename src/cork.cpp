#include "util/ThreadPool.h"


namespace Cork 
{
    void Shutdown( void )
    {
        ThreadPool::getPool().Shutdown();
    }
}
