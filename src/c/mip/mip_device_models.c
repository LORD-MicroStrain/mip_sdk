#include "mip_device_models.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
///@brief Parses a string into a model number.
///
///@param model_or_serial Device model or serial number.
///
///@returns The model number. Note that it may not be listed in the enum (e.g.
///         if it's a new model released after this version of source code).
///
mip_model_number get_model_from_string(const char* model_or_serial)
{
    unsigned int start_index = 0;

    // The model number is just the portion of the serial or model string before the dot or dash
    // serial and model number fields are 16 chars
    unsigned int i = 0;
    for (; model_or_serial[i] != '\0'; i++)
    {
        // Unsigned is important. Passing a negative value to isdigit or
        // isspace is undefined behavior and can trigger assertions.
        const unsigned char c = model_or_serial[i];

        if (!isdigit(c))
        {
            if (c == '.' || c == '-')
                break;

            if (isspace(c) && (start_index == i))
                start_index++;
            else
                return MODEL_UNKNOWN;
        }
    }

    // Must get at least 4 digits.
    if (i < start_index + 4)
        return MODEL_UNKNOWN;

    return (mip_model_number)atoi(model_or_serial + start_index);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Converts a model number to the product's model name.
///
///@param model
///
///@returns A string containing the (approximate) model name.
///
const char* get_model_name_from_number(mip_model_number model)
{
    // TODO: Update all model strings to use - instead of _
    switch (model)
    {
    // Gen 3
    case MODEL_3DM_DH3:           return "3DM-DH3";
    case MODEL_3DM_GX3_25:        return "3DM-GX3-25";
    case MODEL_3DM_GX3_35:        return "3DM-GX3-35";
    case MODEL_3DM_GX3_15:        return "3DM-GX3-15";
    case MODEL_3DM_GX3_45:        return "3DM-GX3-45";
    // Gen 4
    case MODEL_3DM_RQ1_45_LT:     return "3DM-RQ1-45_LT";
    case MODEL_3DM_GX4_15:        return "3DM-GX4-15";
    case MODEL_3DM_GX4_25:        return "3DM-GX4-25";
    case MODEL_3DM_GX4_45:        return "3DM-GX4-45";
    case MODEL_3DM_RQ1_45_ST:     return "3DM-RQ1-45_ST";
    case MODEL_3DM_GQ4_45:        return "3DM-GQ4-45";
    // Gen 5
    case MODEL_3DM_MV5_AR:        return "3DM-MV5-AR";
    case MODEL_3DM_GX5_GNSS_INS:  return "3DM-GX5-GNSS/INS";
    case MODEL_3DM_GX5_GNSS_AHRS: return "3DM-GX5-35"; // GNSS/AHRS
    case MODEL_3DM_GX5_AHRS:      return "3DM-GX5-25"; // AHRS
    case MODEL_3DM_GX5_AR:        return "3DM-GX5-15"; // AR
    case MODEL_3DM_GX5_IMU:       return "3DM-GX5-10"; // IMU
    case MODEL_3DM_CV5_AHRS:      return "3DM-CV5-25"; // AHRS
    case MODEL_3DM_CV5_AR:        return "3DM-CV5-15"; // AR
    case MODEL_3DM_CV5_IMU:       return "3DM-CV5-10"; // IMU
    case MODEL_3DM_CX5_GNSS_INS:  return "3DM-CX5-45"; // GNSS/INS
    case MODEL_3DM_CX5_GNSS_AHRS: return "3DM-CX5-35"; // GNSS/AHRS
    case MODEL_3DM_CX5_AHRS:      return "3DM-CX5-25"; // AHRS
    case MODEL_3DM_CX5_AR:        return "3DM-CX5-15"; // AR
    case MODEL_3DM_CX5_IMU:       return "3DM-CX5-10"; // IMU
    case MODEL_3DM_CL5_IMU:       return "3DM-CL5-10"; // IMU
    case MODEL_3DM_CL5_AR:        return "3DM-CL5-15"; // AR
    case MODEL_3DM_CL5_AHRS:      return "3DM-CL5-25"; // AHRS
    // Gen 7
    case MODEL_3DM_GQ7_GNSS_INS:  return "3DM-GQ7"; // GNSS/INS
    case MODEL_3DM_RTK:           return "3DM-RTK";
    case MODEL_3DM_CV7_AR:        return "3DM-CV7-AR";
    case MODEL_3DM_CV7_AHRS:      return "3DM-CV7-AHRS";
    case MODEL_3DM_CV7_INS:       return "3DM-CV7-INS";
    case MODEL_3DM_CV7_GNSS_INS:  return "3DM-CV7-GNSS/INS";
    case MODEL_3DM_GV7_AR:        return "3DM-GV7-AR";
    case MODEL_3DM_GV7_AHRS:      return "3DM-GV7-AHRS";
    case MODEL_3DM_GV7_INS:       return "3DM-GV7-INS";

    default:
    case MODEL_UNKNOWN: return "";
    }
}

#ifdef __cplusplus
} // extern "C"
} // namespace C
} // namespace mip
#endif // __cplusplus
