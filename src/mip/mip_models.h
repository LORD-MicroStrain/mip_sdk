#pragma once

#ifdef __cplusplus
#include <string>
#include <map>

namespace mip {
#endif // __cplusplus

    enum Model
    {
        MODEL_UNKNOWN       = 0,
        MODEL_3DM_DH3       = 6219, // 3DM-DH3
        MODEL_3DM_GX3_15    = 6227, // 3DM-GX3-15
        MODEL_3DM_GX3_25    = 6223, // 3DM-GX3-25
        MODEL_3DM_GX3_35    = 6225, // 3DM-GX3-35
        MODEL_3DM_GX3_45    = 6228, // 3DM-GX3-45
        MODEL_3DM_RQ1_45_LT = 6232, // 3DM-RQ1-45-LT
        MODEL_3DM_GX4_15    = 6233, // 3DM-GX4-15
        MODEL_3DM_GX4_25    = 6234, // 3DM-GX4-25
        MODEL_3DM_GX4_45    = 6236, // 3DM-GX4-45
        MODEL_3DM_RQ1_45_ST = 6239, // 3DM-RQ1-45-ST
        MODEL_3DM_GX5_10    = 6255, // 3DM-GX5-10
        MODEL_3DM_GX5_15    = 6254, // 3DM-GX5-15
        MODEL_3DM_GX5_25    = 6253, // 3DM-GX5-25
        MODEL_3DM_GX5_35    = 6252, // 3DM-GX5-35
        MODEL_3DM_GX5_45    = 6251, // 3DM-GX5-45
        MODEL_3DM_CV5_10    = 6259, // 3DM-CV5-10
        MODEL_3DM_CV5_15    = 6258, // 3DM-CV5-15
        MODEL_3DM_CV5_25    = 6257, // 3DM-CV5-25
        MODEL_3DM_CV5_45    = 6256, // 3DM-CV5-45
        MODEL_3DM_GQ4_45    = 6250, // 3DM-GQ4-45
        MODEL_3DM_CX5_45    = 6271, // 3DM-CX5-45
        MODEL_3DM_CX5_35    = 6272, // 3DM-CX5-35
        MODEL_3DM_CX5_25    = 6273, // 3DM-CX5-25
        MODEL_3DM_CX5_15    = 6274, // 3DM-CX5-15
        MODEL_3DM_CX5_10    = 6275, // 3DM-CX5-10
        MODEL_3DM_CL5_15    = 6280, // 3DM-CL5-15
        MODEL_3DM_CL5_25    = 6281, // 3DM-CL5-25
        MODEL_3DM_GQ7       = 6284, // 3DM-GQ7
        MODEL_3DM_RTK       = 6285, // 3DM-RTK
        MODEL_3DM_CV7_AHRS  = 6286, // 3DM-CV7-AHRS
        MODEL_3DM_CV7_AR    = 6287, // 3DM-CV7-AR
        MODEL_3DM_GV7_AHRS  = 6288, // 3DM-GV7-AHRS
        MODEL_3DM_GV7_AR    = 6289, // 3DM-GV7-AR
        MODEL_3DM_GV7_INS   = 6290, // 3DM-GV7-INS
        MODEL_3DM_CV7_INS   = 6291  // 3DM-CV7-INS
    };

    enum Model getModelFromSerial(const char* serial);
    enum Model getModelFromModelString(const char* model);

#ifdef __cplusplus
    std::string getModelNameFromSerial(const char* serial);
} // namespace mip
#endif // __cplusplus