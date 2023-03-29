#pragma once

#ifdef MIP_ENABLE_THREADING

#ifdef WIN32
    #define NOMINMAX
    #include <windows.h>

#else
    #include <pthread.h>

#endif

#ifdef __cplusplus
namespace mip {
namespace C {
#endif


typedef struct mip_mutex_type
{
#ifdef WIN32
    HANDLE         _handle;
#else
    pthread_mutex_t _mutex;
#endif
} mip_mutex_type;

void mip_mutex_init(mip_mutex_type* mutex);
void mip_mutex_lock(mip_mutex_type* mutex);
void mip_mutex_unlock(mip_mutex_type* mutex);
void mip_mutex_deinit(mip_mutex_type* mutex);

#define MIP_MUTEX_INIT(mutex) mip_mutex_init  (mutex)
#define MIP_MUTEX_LOCK(mutex) mip_mutex_lock  (mutex)
#define MIP_MUTEX_UNLOCK(mutex) mip_mutex_unlock(mutex)
#define MIP_MUTEX_DEINIT(mutex) mip_mutex_deinit(mutex)

typedef struct mip_thread_signal
{
    mip_mutex_type mutex;

#ifdef WIN32
#else
    pthread_cond_t _cond;
#endif
} mip_thread_signal;

void mip_thread_signal_init(mip_thread_signal* sig);
void mip_thread_signal_deinit(mip_thread_signal* sig);
void mip_thread_wait(mip_thread_signal* sig);
void mip_thread_notify(mip_thread_signal* sig);

#define MIP_THREAD_SIGNAL_INIT(sig)   mip_thread_signal_init(sig)
#define MIP_THREAD_SIGNAL_DEINIT(sig) mip_thread_signal_deinit(sig)
#define MIP_THREAD_WAIT(sig)          mip_thread_wait(sig)
#define MIP_THREAD_NOTIFY(sig)        mip_thread_notify(sig)


#else // MIP_ENABLE_THREADING

#define MIP_MUTEX_INIT(mutex)
#define MIP_MUTEX_LOCK(mutex)
#define MIP_MUTEX_UNLOCK(mutex)
#define MIP_MUTEX_DEINIT(mutex)

#define MIP_THREAD_SIGNAL_INIT(sig)
#define MIP_THREAD_SIGNAL_DEINIT(sig)
#define MIP_THREAD_WAIT(sig)
#define MIP_THREAD_NOTIFY(sig)

#endif // MIP_ENABLE_THREADING

#ifdef __cplusplus
} // namespace C
} // namespace mip
#endif

