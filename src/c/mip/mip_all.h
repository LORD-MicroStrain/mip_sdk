#pragma once

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip  MIP
///@{
///
///@defgroup mip_c MIP C API
///@brief This module contains functions and classes for communicating with a
///       MIP device in %C.
///
///@}
////////////////////////////////////////////////////////////////////////////////


// MIP Core
#include "mip_cmdqueue.h"
#include "mip_descriptors.h"
#include "mip_dispatch.h"
#include "mip_field.h"
#include "mip_interface.h"
#include "mip_offsets.h"
#include "mip_packet.h"
#include "mip_parser.h"
#include "mip_result.h"
#include "mip_serialization.h"
#include "mip_types.h"
#include "mip_version.h"

// MIP Commands
#include "definitions/commands_3dm.h"
#include "definitions/commands_aiding.h"
#include "definitions/commands_base.h"
#include "definitions/commands_filter.h"
#include "definitions/commands_gnss.h"
#include "definitions/commands_rtk.h"
#include "definitions/commands_system.h"

// MIP Data
#include "definitions/data_filter.h"
#include "definitions/data_gnss.h"
#include "definitions/data_sensor.h"
#include "definitions/data_shared.h"
#include "definitions/data_system.h"
