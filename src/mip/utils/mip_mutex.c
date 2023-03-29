
#include "mip_mutex.h"

#include "mip/mip_logging.h"

#include <assert.h>

#ifndef WIN32
    #include <string.h>
#endif

#ifdef MIP_ENABLE_THREADING

//
// Mutex
//

void mip_mutex_init(mip_mutex_type* mutex)
{
#ifdef WIN32
    // Todo: CriticalSections may be a better choice
    mutex->_handle = CreateMutex(NULL, FALSE, NULL);

    if(mutex->_handle == INVALID_HANDLE_VALUE)
        MIP_LOG_ERROR("Failed to initialize mutex: %d\n", GetLastError());

#else
    pthread_mutexattr_t attr;
    int result;

    result = pthread_mutexattr_init(&attr);
    assert(result == 0);

    // A recursive mutex is required by mip_interface_start_command_packet.
    result = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    assert(result == 0);

    result = pthread_mutex_init(&mutex->_mutex, &attr);

    pthread_mutexattr_destroy(&attr);

    //result = pthread_mutex_init(&mutex->_mutex, NULL);

    if( result != 0 )
        MIP_LOG_ERROR("Failed to initialize mutex: %s (%d)\n", strerror(result), result);
#endif
}


void mip_mutex_deinit(mip_mutex_type* mutex)
{
#ifdef WIN32

    CloseHandle(mutex->_handle);
    mutex->_handle = INVALID_HANDLE_VALUE;

#else

    int result = pthread_mutex_destroy(&mutex->_mutex);

    if( result != 0 )
        MIP_LOG_ERROR("Failed to destroy mutex: %s (%d)\n", strerror(result), result);

#endif
}


void mip_mutex_lock(mip_mutex_type* mutex)
{
#ifdef WIN32

    DWORD result = WaitForSingleObject(mutex->_handle, INFINITE);

    if(result != WAIT_OBJECT_0)
    {
        if( result == WAIT_ABANDONED )
            MIP_LOG_ERROR("Mutex was abandoned!\n");
        else
            MIP_LOG_ERROR("Failed to lock mutex: %d\n", GetLastError());
    }

#else

    int result = pthread_mutex_lock(&mutex->_mutex);

    if( result != 0 )
        MIP_LOG_ERROR("Failed to lock mutex: %s (%d)\n", strerror(result), result);

#endif
}


void mip_mutex_unlock(mip_mutex_type* mutex)
{
#ifdef WIN32

    if( !ReleaseMutex(mutex->_handle) )
        MIP_LOG_ERROR("Failed to unlock mutex: %d\n", GetLastError());

#else

    int result = pthread_mutex_unlock(&mutex->_mutex);

    if( result != 0 )
        MIP_LOG_ERROR("Failed to unlock mutex: %s (%d)\n", strerror(result), result);

#endif
}

//
// Thread Signaling
//

void mip_thread_signal_init(mip_thread_signal* sig)
{
    mip_mutex_init(&sig->_mutex);

#ifdef WIN32
#else
    int result = pthread_cond_init(&sig->_cond);

    if( result != 0 )
        MIP_LOG_ERROR("Failed to initialize condition variable: %s (%d)\n", strerr(result), result);
#endif
}

void mip_thread_signal_deinit(mip_thread_signal* sig)
{
#ifdef WIN32



#else

    int result = pthread_cond_destroy(&sig->_cond);

    if( result != 0 )
        MIP_LOG_ERROR("Failed to destroy condition variable: %s (%d)\n", strerr(result), result);

#endif

    mip_mutex_deinit(&sig->_mutex);
}

void mip_thread_wait(mip_thread_signal* sig)
{
#ifdef WIN32



#else

    pthread_cond_wait(&sig->_cond, &sig->_mutex._mutex);

#endif
}

void mip_thread_notify(mip_thread_signal* sig)
{
#ifdef WIN32



#else



#endif
}



#endif // MIP_ENABLE_THREADING
