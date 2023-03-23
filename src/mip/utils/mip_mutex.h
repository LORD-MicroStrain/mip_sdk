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

#else // MIP_ENABLE_THREADING

#define MIP_MUTEX_INIT(mutex)
#define MIP_MUTEX_LOCK(mutex)
#define MIP_MUTEX_UNLOCK(mutex)
#define MIP_MUTEX_DEINIT(mutex)

#endif // MIP_ENABLE_THREADING

#ifdef __cplusplus
} // namespace C
} // namespace mip
#endif

