#pragma once

#include <microstrain/embedded_time.h>

#include <chrono>

namespace microstrain
{
    using EmbeddedTimestamp = C::microstrain_embedded_timestamp;
    using EmbeddedTimeout   = C::microstrain_embedded_timeout;

    ////////////////////////////////////////////////////////////////////////////////
    /// @brief Gets the current system timestamp in milliseconds
    ///
    /// @details Provides system time measurement using std::chrono for milliseconds
    ///          since system clock epoch.
    ///
    /// @return Current system time in milliseconds since epoch
    ///
    static inline EmbeddedTimestamp getCurrentTimestamp()
    {
        const std::chrono::nanoseconds timeSinceEpoch = std::chrono::system_clock::now().time_since_epoch();
        return static_cast<EmbeddedTimestamp>(
            std::chrono::duration_cast<std::chrono::milliseconds>(timeSinceEpoch).count());
    }
} // namespace microstrain
