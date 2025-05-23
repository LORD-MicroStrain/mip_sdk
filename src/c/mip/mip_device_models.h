#pragma once

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {
#endif // __cplusplus

enum mip_model_number
{
    MODEL_UNKNOWN           = 0,
    // Gen 3
    MODEL_3DM_DH3           = 6219,
    MODEL_3DM_GX3_25        = 6223,
    MODEL_3DM_GX3_35        = 6225,
    MODEL_3DM_GX3_15        = 6227,
    MODEL_3DM_GX3_45        = 6228,
    // Gen 4
    MODEL_3DM_RQ1_45_LT     = 6232,
    MODEL_3DM_GX4_15        = 6233,
    MODEL_3DM_GX4_25        = 6234,
    MODEL_3DM_GX4_45        = 6236,
    MODEL_3DM_RQ1_45_ST     = 6239,
    MODEL_3DM_GQ4_45        = 6250,
    // Gen 5
    MODEL_3DM_MV5_AR        = 6243, // 3DM_MV5_15
    MODEL_3DM_GX5_GNSS_INS  = 6251, // 3DM-GX5-45
    MODEL_3DM_GX5_GNSS_AHRS = 6252, // 3DM-GX5-35
    MODEL_3DM_GX5_AHRS      = 6253, // 3DM-GX5-25
    MODEL_3DM_GX5_AR        = 6254, // 3DM-GX5-15
    MODEL_3DM_GX5_IMU       = 6255, // 3DM-GX5-10
    MODEL_3DM_CV5_AHRS      = 6257, // 3DM-CV5-25
    MODEL_3DM_CV5_AR        = 6258, // 3DM-CV5-15
    MODEL_3DM_CV5_IMU       = 6259, // 3DM-CV5-10
    MODEL_3DM_CX5_GNSS_INS  = 6271, // 3DM-CX5-45
    MODEL_3DM_CX5_GNSS_AHRS = 6272, // 3DM-CX5-35
    MODEL_3DM_CX5_AHRS      = 6273, // 3DM-CX5-25
    MODEL_3DM_CX5_AR        = 6274, // 3DM-CX5-15
    MODEL_3DM_CX5_IMU       = 6275, // 3DM-CX5-10
    MODEL_3DM_CL5_IMU       = 6279, // 3DM-CL5-10
    MODEL_3DM_CL5_AR        = 6280, // 3DM-CL5-15
    MODEL_3DM_CL5_AHRS      = 6281, // 3DM-CL5-25
    // Gen 7
    MODEL_3DM_GQ7_GNSS_INS  = 6284,
    MODEL_3DM_RTK           = 6285,
    MODEL_3DM_CV7_AHRS      = 6286,
    MODEL_3DM_CV7_AR        = 6287,
    MODEL_3DM_GV7_AHRS      = 6288,
    MODEL_3DM_GV7_AR        = 6289,
    MODEL_3DM_GV7_INS       = 6290,
    MODEL_3DM_CV7_INS       = 6291,
    MODEL_3DM_CV7_GNSS_INS  = 6292
};
#ifndef __cplusplus
typedef enum mip_model_number mip_model_number;
#endif // __cplusplus

mip_model_number get_model_from_string(const char* model_or_serial);
const char* get_model_name_from_number(mip_model_number model);

#ifdef __cplusplus
} // extern "C"
} // namespace C

using ModelNumber = C::mip_model_number;

inline ModelNumber getModelFromString(const char* model_or_serial) { return C::get_model_from_string(model_or_serial); }
inline const char* getModelNameFromNumber(ModelNumber model) { return C::get_model_name_from_number(model); }
} // namespace mip
#endif // __cplusplus
