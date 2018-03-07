#pragma once
#include "packetdef.h"

#include <cstdint>

static int const CRTP_MAXSIZE = 31;
#define CHECKSIZE(s) static_assert(sizeof(s) <= CRTP_MAXSIZE, #s " packet is too large");

static int const CRTP_MAXSIZE_RESPONSE = 32;
#define CHECKSIZE_RESPONSE(s) static_assert(sizeof(s) <= CRTP_MAXSIZE_RESPONSE, #s " packet is too large");

// Header
struct crtp
{
  constexpr crtp(uint8_t port, uint8_t channel)
    : channel(channel)
    , link(3)
    , port(port)
  {
  }

  crtp(uint8_t byte)
  {
    channel = (byte >> 0) & 0x3;
    link    = (byte >> 2) & 0x3;
    port    = (byte >> 4) & 0xF;
  }

  bool operator==(const crtp& other) const {
    return channel == other.channel && port == other.port;
  }

  uint8_t channel:2;
  uint8_t link:2;
  uint8_t port:4;
} __attribute__((packed));

// Port 0 (Console)
struct crtpConsoleResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return crtp(response.data[0]) == crtp(0, 0);
    }

    crtp header;
    char text[31];
};
CHECKSIZE_RESPONSE(crtpConsoleResponse)

// Port 2 (Parameters)

struct crtpParamTocGetItemResponse;
struct crtpParamTocGetItemRequest
{
  crtpParamTocGetItemRequest(
    uint8_t id)
    : header(2, 0)
    , command(0)
    , id(id)
  {
  }

  bool operator==(const crtpParamTocGetItemRequest& other) const {
    return header == other.header && command == other.command && id == other.id;
  }

  typedef crtpParamTocGetItemResponse Response;

  const crtp header;
  const uint8_t command;
  uint8_t id;
} __attribute__((packed));
CHECKSIZE(crtpParamTocGetItemRequest)

struct crtpParamTocGetItemResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size > 5 &&
           crtp(response.data[0]) == crtp(2, 0) &&
           response.data[1] == 0;
  }

  crtpParamTocGetItemRequest request;
  uint8_t length:2; // one of ParamLength
  uint8_t type:1;   // one of ParamType
  uint8_t sign:1;   // one of ParamSign
  uint8_t res0:2;   // reserved
  uint8_t readonly:1;
  uint8_t group:1;  // one of ParamGroup
  char text[28]; // group, name
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamTocGetItemResponse)

struct crtpParamTocGetInfoResponse;
struct crtpParamTocGetInfoRequest
{
  crtpParamTocGetInfoRequest()
    : header(2, 0)
    , command(1)
  {
  }

  bool operator==(const crtpParamTocGetInfoRequest& other) const {
    return header == other.header && command == other.command;
  }

  typedef crtpParamTocGetInfoResponse Response;

  const crtp header;
  const uint8_t command;
} __attribute__((packed));
CHECKSIZE(crtpParamTocGetInfoRequest)

struct crtpParamTocGetInfoResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size == 7 &&
           crtp(response.data[0]) == crtp(2, 0) &&
           response.data[1] == 1;
  }

  crtpParamTocGetInfoRequest request;
  uint8_t numParam;
  uint32_t crc;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamTocGetInfoResponse)

struct crtpParamValueResponse;
struct crtpParamReadRequest
{
  crtpParamReadRequest(
    uint8_t id)
    : header(2, 1)
    , id(id)
  {
  }

  bool operator==(const crtpParamReadRequest& other) const {
    return header == other.header && id == other.id;
  }

  typedef crtpParamValueResponse Response;

  const crtp header;
  const uint8_t id;
} __attribute__((packed));
CHECKSIZE(crtpParamReadRequest)

template <class T>
struct crtpParamWriteRequest
{
  crtpParamWriteRequest(
    uint8_t id,
    const T& value)
    : header(2, 2)
    , id(id)
    , value(value)
    {
    }

    const crtp header;
    const uint8_t id;
    const T value;
} __attribute__((packed));
CHECKSIZE(crtpParamWriteRequest<double>) // largest kind of param

struct crtpParamValueResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size > 2 &&
           (crtp(response.data[0]) == crtp(2, 1) ||
            crtp(response.data[0]) == crtp(2, 2));
  }

  crtpParamReadRequest request;
  union {
    uint8_t valueUint8;
    int8_t valueInt8;
    uint16_t valueUint16;
    int16_t valueInt16;
    uint32_t valueUint32;
    int32_t valueInt32;
    float valueFloat;
  };
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamValueResponse)

template <class T>
struct crtpParamWriteBroadcastRequest
{
  crtpParamWriteBroadcastRequest(
    uint8_t group,
    uint8_t id,
    const T& value)
    : header(2, 3)
    , command(1)
    , group(group)
    , id(id)
    , value(value)
    {
    }

    const crtp header;
    const uint8_t command;
    const uint8_t group;
    const uint8_t id;
    const T value;
} __attribute__((packed));
CHECKSIZE(crtpParamWriteBroadcastRequest<double>) // largest kind of param

// Port 3 (Commander)

struct crtpSetpointRequest
{
  crtpSetpointRequest(
    float roll,
    float pitch,
    float yawrate,
    uint16_t thrust)
    : header(0x03, 0)
    , roll(roll)
    , pitch(pitch)
    , yawrate(yawrate)
    , thrust(thrust)
  {
  }
  const crtp header;
  float roll;
  float pitch;
  float yawrate;
  uint16_t thrust;
}  __attribute__((packed));
CHECKSIZE(crtpSetpointRequest)

// Port 4 (Memory access)

// Port 5 (Data logging)

struct crtpLogGetInfoResponse;
struct crtpLogGetInfoRequest
{
  crtpLogGetInfoRequest()
    : header(5, 0)
    , command(1)
    {
    }

  bool operator==(const crtpLogGetInfoRequest& other) const {
    return header == other.header && command == other.command;
  }

  typedef crtpLogGetInfoResponse Response;

  const crtp header;
  const uint8_t command;
} __attribute__((packed));
CHECKSIZE(crtpLogGetInfoRequest)

struct crtpLogGetInfoResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size == 9 &&
           crtp(response.data[0]) == crtp(5, 0) &&
           response.data[1] == 1;
  }

  crtpLogGetInfoRequest request;
  // Number of log items contained in the log table of content
  uint8_t log_len;
  // CRC values of the log TOC memory content. This is a fingerprint of the copter build that can be used to cache the TOC
  uint32_t log_crc;
  // Maximum number of log packets that can be programmed in the copter
  uint8_t log_max_packet;
  // Maximum number of operation programmable in the copter. An operation is one log variable retrieval programming
  uint8_t log_max_ops;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogGetInfoResponse)

struct crtpLogGetItemResponse;
struct crtpLogGetItemRequest
{
  crtpLogGetItemRequest(uint8_t id)
    : header(5, 0)
    , command(0)
    , id(id)
  {
  }

  bool operator==(const crtpLogGetItemRequest& other) const {
    return header == other.header && command == other.command && id == other.id;
  }

  typedef crtpLogGetItemResponse Response;

  const crtp header;
  const uint8_t command;
  uint8_t id;
} __attribute__((packed));
CHECKSIZE(crtpLogGetItemRequest)

struct crtpLogGetItemResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 5 &&
             crtp(response.data[0]) == crtp(5, 0) &&
             response.data[1] == 0;
    }

    crtpLogGetItemRequest request;
    uint8_t type;
    char text[28]; // group, name
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogGetItemResponse)

struct logBlockItem {
  uint8_t logType;
  uint8_t id;
} __attribute__((packed));

struct crtpLogCreateBlockRequest
{
  crtpLogCreateBlockRequest()
  : header(5, 1)
  , command(0)
  {
  }

  const crtp header;
  const uint8_t command;
  uint8_t id;
  logBlockItem items[14]; // TODO: FIX IN FIRMWARE!
} __attribute__((packed));
CHECKSIZE(crtpLogCreateBlockRequest)

// struct logAppendBlockRequest
// {
//   logAppendBlockRequest()
//     : header(5, 1)
//     , command(1)
//     {
//     }

//     const crtp header;
//     const uint8_t command;
//     uint8_t id;
//     logBlockItem items[16];
// } __attribute__((packed));

// struct logDeleteBlockRequest
// {
//   logDeleteBlockRequest()
//     : header(5, 1)
//     , command(2)
//     {
//     }

//     const crtp header;
//     const uint8_t command;
//     uint8_t id;
// } __attribute__((packed));

struct crtpLogStartRequest
{
  crtpLogStartRequest(
    uint8_t id,
    uint8_t period)
    : header(5, 1)
    , command(3)
    , id(id)
    , period(period)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t id;
    uint8_t period; // in increments of 10ms
} __attribute__((packed));
CHECKSIZE(crtpLogStartRequest)

struct crtpLogStopRequest
{
  crtpLogStopRequest(
    uint8_t id)
    : header(5, 1)
    , command(4)
    , id(id)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t id;
} __attribute__((packed));
CHECKSIZE(crtpLogStopRequest)

struct crtpLogResetRequest
{
  crtpLogResetRequest()
    : header(5, 1)
    , command(5)
    {
    }

    const crtp header;
    const uint8_t command;
} __attribute__((packed));
CHECKSIZE(crtpLogResetRequest)

struct crtpLogStart2Request
{
  crtpLogStart2Request(
    uint8_t id,
    uint16_t period)
    : header(5, 1)
    , command(3)
    , id(id)
    , period(period)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t id;
    uint16_t period; // in increments of 1ms
} __attribute__((packed));

enum crtpLogControlResult {
  crtpLogControlResultOk            = 0,
  crtpLogControlResultOutOfMemory   = 12, // ENOMEM
  crtpLogControlResultCmdNotFound   = 8,  // ENOEXEC
  crtpLogControlResultWrongBlockId  = 2,  // ENOENT
  crtpLogControlResultBlockTooLarge = 7,  // E2BIG
  crtpLogControlResultBlockExists   = 17, // EEXIST

};

struct crtpLogControlResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size == 4 &&
             crtp(response.data[0]) == crtp(5, 1);
    }

    crtp header;
    uint8_t command;
    uint8_t requestByte1;
    uint8_t result; // one of crtpLogControlResult
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogControlResponse)

struct crtpLogDataResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 4 &&
             crtp(response.data[0]) == crtp(5, 2);
    }

    crtp header;
    uint8_t blockId;
    uint8_t timestampLo;
    uint16_t timestampHi;
    uint8_t data[26];
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogDataResponse)

// Port 11 (PosExtBringup)

typedef uint16_t fp16_t;

struct crtpPosExtBringup
{
  crtpPosExtBringup()
    : header(11, 1)
    {
      data.pose[0].id = 0;
      data.pose[1].id = 0;
    }

    const crtp header;
    struct data_vicon data;
} __attribute__((packed));
CHECKSIZE(crtpPosExtBringup)

struct crtpPacketDropTest
{
  crtpPacketDropTest(uint64_t seq)
    : header(11, 1)
    {
      data.seq = seq;
    }

    const crtp header;
    struct data_packed_drops data;
} __attribute__((packed));
CHECKSIZE(crtpPacketDropTest)

// Port 12 (PosExt)

struct crtpPosExt
{
  crtpPosExt()
    : header(12, 1)
    {
    }

    const crtp header;
    struct {
      uint8_t id;
      fp16_t x; // m
      fp16_t y; // m
      fp16_t z; // m
      fp16_t yaw; // deg
    } position[3];
} __attribute__((packed));
CHECKSIZE(crtpPosExt)

// Port 14 (Trajectory)

struct crtpTrajectoryResetRequest
{
  crtpTrajectoryResetRequest()
    : header(14, 1)
    , command(COMMAND_RESET_POLY)
    {
    }

    const crtp header;
    const uint8_t command;
} __attribute__((packed));
CHECKSIZE(crtpTrajectoryResetRequest)

struct crtpTrajectoryAddRequest
{
  crtpTrajectoryAddRequest()
    : header(14, 1)
    , command(COMMAND_ADD_POLY)
    {
    }

    const crtp header;
    const uint8_t command;
    struct data_add_poly data;
} __attribute__((packed));
CHECKSIZE(crtpTrajectoryAddRequest)

struct crtpTrajectoryStartRequest
{
  crtpTrajectoryStartRequest(
    uint8_t group,
    bool reversed)
    : header(14, 1)
    , command(COMMAND_START_POLY)
    {
      data.group = group;
      data.reversed = reversed;
    }

    const crtp header;
    const uint8_t command;
    struct data_start_poly data;
} __attribute__((packed));
CHECKSIZE(crtpTrajectoryStartRequest)

struct crtpTrajectoryTakeoffRequest
{
  crtpTrajectoryTakeoffRequest(
    uint8_t group,
    float height,
    uint16_t time_from_start)
    : header(14, 1)
    , command(COMMAND_TAKEOFF)
    {
      data.group = group;
      data.height = height;
      data.time_from_start = time_from_start;
    }

    const crtp header;
    const uint8_t command;
    struct data_takeoff data;
} __attribute__((packed));
CHECKSIZE(crtpTrajectoryTakeoffRequest)

struct crtpTrajectoryLandRequest
{
  crtpTrajectoryLandRequest(
    uint8_t group,
    float height,
    uint16_t time_from_start)
    : header(14, 1)
    , command(COMMAND_LAND)
    {
      data.group = group;
      data.height = height;
      data.time_from_start = time_from_start;
    }

    const crtp header;
    const uint8_t command;
    struct data_land data;
} __attribute__((packed));
CHECKSIZE(crtpTrajectoryLandRequest)

struct crtpTrajectoryHoverRequest
{
  crtpTrajectoryHoverRequest(
    float x,
    float y,
    float z,
    float yaw,
    float duration)
    : header(14, 1)
    , command(COMMAND_HOVER)
    {
        data.x = x; data.y = y; data.z = z; data.yaw = yaw;
        data.duration = duration;
    }

    const crtp header;
    const uint8_t command;
    struct data_hover data;
} __attribute__((packed));

struct crtpTrajectoryStartEllipseRequest
{
  crtpTrajectoryStartEllipseRequest(
    uint8_t group)
    : header(14, 1)
    , command(COMMAND_START_ELLIPSE)
    {
      data.group = group;
    }

    const crtp header;
    const uint8_t command;
    struct data_start_ellipse data;
} __attribute__((packed));

struct crtpTrajectoryGoHomeRequest
{
  crtpTrajectoryGoHomeRequest(
    uint8_t group)
    : header(14, 1)
    , command(COMMAND_GOHOME)
    {
      data.group = group;
    }

    const crtp header;
    const uint8_t command;
    struct data_gohome data;
} __attribute__((packed));
CHECKSIZE(crtpTrajectoryHoverRequest)

struct crtpTrajectorySetEllipseRequest
{
  crtpTrajectorySetEllipseRequest()
    : header(14, 1)
    , command(COMMAND_SET_ELLIPSE)
    {
    }

    const crtp header;
    const uint8_t command;
    struct data_set_ellipse data;
} __attribute__((packed));
CHECKSIZE(crtpTrajectorySetEllipseRequest)

struct crtpTrajectoryStartCannedRequest
{
  crtpTrajectoryStartCannedRequest(
    uint8_t group,
    uint16_t trajectory,
    float timescale)
    : header(14, 1)
    , command(COMMAND_START_CANNED_TRAJECTORY)
    {
      data.group = group;
      data.trajectory = trajectory;
      data.timescale = timescale;
    }

    const crtp header;
    const uint8_t command;
    struct data_start_canned_trajectory data;
} __attribute__((packed));
CHECKSIZE(crtpTrajectoryStartCannedRequest)

struct crtpTrajectorySetGroupRequest
{
  crtpTrajectorySetGroupRequest(
    uint8_t group)
    : header(14, 1)
    , command(COMMAND_SET_GROUP)
    {
      data.group = group;
    }

    const crtp header;
    const uint8_t command;
    struct data_set_group data;
} __attribute__((packed));
CHECKSIZE(crtpTrajectorySetGroupRequest)

// struct crtpTrajectoryStateRequest
// {
//   crtpTrajectoryStateRequest(uint8_t state)
//     : header(14, 1)
//     , command(3)
//     , state(state)
//     {
//     }

//     const crtp header;
//     const uint8_t command;
//     const uint8_t state;
// } __attribute__((packed));

struct crtpTrajectoryResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size == 5
        && crtp(response.data[0]) == crtp(14, 1);
    }

    const crtp header;
    const uint8_t command;
    const uint8_t cmd1;
    const uint8_t cmd2;
    const uint8_t result;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpTrajectoryResponse)

// Port 13 (Platform)

// The crazyflie-nrf firmware sends empty packets with the signal strength, if nothing else is in the queue
struct crtpPlatformRSSIAck
{
    static bool match(const Crazyradio::Ack& response) {
      return crtp(response.data[0]) == crtp(15, 3);
    }

    crtp header;
    uint8_t reserved;
    uint8_t rssi;
};
