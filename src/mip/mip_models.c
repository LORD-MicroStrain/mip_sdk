#include "mip_models.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
#endif // __cplusplus

    enum Model getModelFromSerial(const char* serial)
    {
        char model_str[16];
        bool found = false;
        // The model number is just the portion of the serial or model string before the dot or dash
        // serial and model number fields are 16 chars
        for (uint8_t i = 0; i < 16; i++)
        {
            model_str[i] = serial[i];
            if (serial[i] == '.'
                || serial[i] == '-')
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            return MODEL_UNKNOWN;
        }

        return atoi(model_str);
    }

    enum Model getModelFromModelString(const char* model)
    {
        // same logic can be used on either serial or model number string
        // serial:  model.deviceid
        // model:   model-specifier
        return getModelFromSerial(model);
    }

#ifdef __cplusplus

    std::string getModelNameFromSerial(const char* serial)
    {
        enum Model model = getModelFromSerial(serial);
        switch (model)
        {
            case MODEL_3DM_DH3      : return "3DM-DH3";
            case MODEL_3DM_GX3_15   : return "3DM-GX3-15";
            case MODEL_3DM_GX3_25   : return "3DM-GX3-25";
            case MODEL_3DM_GX3_35   : return "3DM-GX3-35";
            case MODEL_3DM_GX3_45   : return "3DM-GX3-45";
            case MODEL_3DM_RQ1_45_LT: return "3DM-RQ1-45_LT";
            case MODEL_3DM_GX4_15   : return "3DM-GX4-15";
            case MODEL_3DM_GX4_25   : return "3DM-GX4-25";
            case MODEL_3DM_GX4_45   : return "3DM-GX4-45";
            case MODEL_3DM_RQ1_45_ST: return "3DM-RQ1-45_ST";
            case MODEL_3DM_GX5_10   : return "3DM-GX5-10";
            case MODEL_3DM_GX5_15   : return "3DM-GX5-15";
            case MODEL_3DM_GX5_25   : return "3DM-GX5-25";
            case MODEL_3DM_GX5_35   : return "3DM-GX5-35";
            case MODEL_3DM_GX5_45   : return "3DM-GX5-45";
            case MODEL_3DM_CV5_10   : return "3DM-CV5-10";
            case MODEL_3DM_CV5_15   : return "3DM-CV5-15";
            case MODEL_3DM_CV5_25   : return "3DM-CV5-25";
            case MODEL_3DM_CV5_45   : return "3DM-CV5-45";
            case MODEL_3DM_GQ4_45   : return "3DM-GQ4-45";
            case MODEL_3DM_CX5_45   : return "3DM-CX5-45";
            case MODEL_3DM_CX5_35   : return "3DM-CX5-35";
            case MODEL_3DM_CX5_25   : return "3DM-CX5-25";
            case MODEL_3DM_CX5_15   : return "3DM-CX5-15";
            case MODEL_3DM_CX5_10   : return "3DM-CX5-10";
            case MODEL_3DM_CL5_15   : return "3DM-CL5-15";
            case MODEL_3DM_CL5_25   : return "3DM-CL5-25";
            case MODEL_3DM_GQ7      : return "3DM-GQ7";
            case MODEL_3DM_RTK      : return "3DM-RTK";
            case MODEL_3DM_CV7_AHRS : return "3DM-CV7-AHRS";
            case MODEL_3DM_CV7_AR   : return "3DM-CV7-AR";
            case MODEL_3DM_GV7_AHRS : return "3DM-GV7-AHRS";
            case MODEL_3DM_GV7_AR   : return "3DM-GV7-AR";
            case MODEL_3DM_GV7_INS  : return "3DM-GV7-INS";
            case MODEL_3DM_CV7_INS  : return "3DM-CV7-INS";

            default:
            case MODEL_UNKNOWN      : return "";
        }
    }
} // namespace mip
#endif // __cplusplus