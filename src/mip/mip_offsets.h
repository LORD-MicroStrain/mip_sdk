#pragma once

#ifdef __cplusplus
namespace mip {
namespace C {
#endif // __cplusplus

enum
{
    MIP_INDEX_SYNC1   = 0,
    MIP_INDEX_SYNC2   = 1,
    MIP_INDEX_DESCSET = 2,
    MIP_INDEX_LENGTH  = 3,
    MIP_INDEX_PAYLOAD = 4,
};

enum
{
    MIP_INDEX_FIELD_LEN     = 0,
    MIP_INDEX_FIELD_DESC    = 1,
    MIP_INDEX_FIELD_PAYLOAD = 2,
};

enum
{
    MIP_HEADER_LENGTH             = 4,
    MIP_CHECKSUM_LENGTH           = 2,
    MIP_PACKET_PAYLOAD_LENGTH_MIN = 0,
    MIP_PACKET_PAYLOAD_LENGTH_MAX = 255,
    MIP_PACKET_LENGTH_MIN         = (MIP_HEADER_LENGTH + MIP_CHECKSUM_LENGTH + MIP_PACKET_PAYLOAD_LENGTH_MIN),
    MIP_PACKET_LENGTH_MAX         = (MIP_HEADER_LENGTH + MIP_CHECKSUM_LENGTH + MIP_PACKET_PAYLOAD_LENGTH_MAX),
};

enum
{
    MIP_FIELD_HEADER_LENGTH      = MIP_INDEX_FIELD_PAYLOAD,
    MIP_FIELD_LENGTH_MIN         = MIP_FIELD_HEADER_LENGTH,
    MIP_FIELD_LENGTH_MAX         = 255,
    MIP_FIELD_PAYLOAD_LENGTH_MAX = (MIP_FIELD_LENGTH_MAX - MIP_FIELD_HEADER_LENGTH),
};

enum
{
    MIP_SYNC1 = 0x75,
    MIP_SYNC2 = 0x65,
};

#ifdef __cplusplus
} // namespace C

static constexpr size_t PACKET_LENGTH_MAX        = C::MIP_PACKET_LENGTH_MAX;
static constexpr size_t PACKET_LENGTH_MIN        = C::MIP_PACKET_LENGTH_MIN;
static constexpr size_t FIELD_PAYLOAD_LENGTH_MAX = C::MIP_FIELD_LENGTH_MAX;

} // namespace mip
#endif // __cplusplus
