#pragma once

#if defined _WIN32
#define MICROSTRAIN_PLATFORM_WINDOWS
#define MICROSTRAIN_PLATFORM_DESKTOP
#elif defined __APPLE__
#define MICROSTRAIN_PLATFORM_APPLE
#define MICROSTRAIN_PLATFORM_DESKTOP
#elif defined __linux__
#define MICROSTRAIN_PLATFORM_LINUX
#define MICROSTRAIN_PLATFORM_DESKTOP
#else
#define MICROSTRAIN_PLATFORM_OTHER
#endif
