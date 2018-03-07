//#include <regex>
#include <mutex>

#include "Crazyflie.h"
#include "crtp.h"
#include "bootloader.h"

#include "Crazyradio.h"
#include "CrazyflieUSB.h"

#include "quatcompress.h" // FROM CF FIRMWARE

#include <iostream>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "num.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

#define MAX_RADIOS 16
#define MAX_USB     4

Crazyradio* g_crazyradios[MAX_RADIOS];
std::mutex g_radioMutex[MAX_RADIOS];

CrazyflieUSB* g_crazyflieUSB[MAX_USB];
std::mutex g_crazyflieusbMutex[MAX_USB];

Logger EmptyLogger;


Crazyflie::Crazyflie(
  const std::string& link_uri,
  Logger& logger)
  : m_radio(nullptr)
  , m_transport(nullptr)
  , m_devId(0)
  , m_channel(0)
  , m_address(0)
  , m_datarate(Crazyradio::Datarate_250KPS)
  , m_logTocEntries()
  , m_logBlockCb()
  , m_paramTocEntries()
  , m_paramValues()
  , m_emptyAckCallback(nullptr)
  , m_linkQualityCallback(nullptr)
  , m_consoleCallback(nullptr)
  , m_lastTrajectoryId(0)
  , m_logger(logger)
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%lx",
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) == 5;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) == 4;
    m_address = 0xE7E7E7E7E7;
  }

  if (success)
  {
    m_channel = channel;
    if (datarate == 250 && datarateType == 'K') {
      m_datarate = Crazyradio::Datarate_250KPS;
    }
    else if (datarate == 1 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_1MPS;
    }
    else if (datarate == 2 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_2MPS;
    }

    if (m_devId >= MAX_RADIOS) {
      throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
    }

    {
      std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
      if (!g_crazyradios[m_devId]) {
        g_crazyradios[m_devId] = new Crazyradio(m_devId);
        // g_crazyradios[m_devId]->setAckEnable(false);
        g_crazyradios[m_devId]->setAckEnable(true);
        g_crazyradios[m_devId]->setArc(0);
      }
    }

    m_radio = g_crazyradios[m_devId];
  }
  else {
    success = std::sscanf(link_uri.c_str(), "usb://%d",
       &m_devId) == 1;

    if (m_devId >= MAX_USB) {
      throw std::runtime_error("This version does not support that many CFs over USB. Adjust MAX_USB and recompile!");
    }

    {
      std::unique_lock<std::mutex> mlock(g_crazyflieusbMutex[m_devId]);
      if (!g_crazyflieUSB[m_devId]) {
        g_crazyflieUSB[m_devId] = new CrazyflieUSB(m_devId);
      }
    }

    m_transport = g_crazyflieUSB[m_devId];
  }

  if (!success) {
    throw std::runtime_error("Uri is not valid!");
  }
}

void Crazyflie::logReset()
{
  crtpLogResetRequest request;
  startBatchRequest();
  addRequest(request, 1);
  handleRequests();
}

// void Crazyflie::sendSetpoint(
//   float roll,
//   float pitch,
//   float yawrate,
//   uint16_t thrust)
// {
//   crtpSetpointRequest request(roll, pitch, yawrate, thrust);
//   sendPacket((const uint8_t*)&request, sizeof(request));
// }

void Crazyflie::sendPing()
{
  uint8_t ping = 0xFF;
  sendPacket(&ping, sizeof(ping));
}

// https://forum.bitcraze.io/viewtopic.php?f=9&t=1488
void Crazyflie::reboot()
{
  const uint8_t reboot_init[] = {0xFF, 0xFE, 0xFF};
  sendPacketOrTimeout(reboot_init, sizeof(reboot_init));

  const uint8_t reboot_to_firmware[] = {0xFF, 0xFE, 0xF0, 0x01};
  sendPacketOrTimeout(reboot_to_firmware, sizeof(reboot_to_firmware));
}

uint64_t Crazyflie::rebootToBootloader()
{
  bootloaderResetInitRequest req(TargetNRF51);
  startBatchRequest();
  addRequest(req, 3);
  handleRequests(/*crtpMode=*/false);
  const bootloaderResetInitResponse* response = getRequestResult<bootloaderResetInitResponse>(0);

  uint64_t result =
      ((uint64_t)response->addr[0] << 0)
    | ((uint64_t)response->addr[1] << 8)
    | ((uint64_t)response->addr[2] << 16)
    | ((uint64_t)response->addr[3] << 24)
    | ((uint64_t)0xb1 << 32);

  const uint8_t reboot_to_bootloader[] = {0xFF, 0xFE, 0xF0, 0x00};
  // for (size_t i = 0; i < 10; ++i) {
  //   if (sendPacket(reboot_to_bootloader, sizeof(reboot_to_bootloader))) {
  //     break;
  //   }
  // }
  sendPacketOrTimeout(reboot_to_bootloader, sizeof(reboot_to_bootloader));

  return result;
}

// needs custom nrf firmware
void Crazyflie::sysoff()
{
  const uint8_t shutdown[] = {0xFF, 0xFE, 0x02};
  sendPacketOrTimeout(shutdown, sizeof(shutdown));
}

void Crazyflie::trySysOff()
{
  const uint8_t shutdown[] = {0xFF, 0xFE, 0x02};
  for (size_t i = 0; i < 10; ++i) {
    if (sendPacket(shutdown, sizeof(shutdown))) {
      break;
    }
  }
}

void Crazyflie::alloff()
{
  const uint8_t shutdown[] = {0xFF, 0xFE, 0x01};
  sendPacketOrTimeout(shutdown, sizeof(shutdown));
}

void Crazyflie::syson()
{
  const uint8_t shutdown[] = {0xFF, 0xFE, 0x03};
  sendPacketOrTimeout(shutdown, sizeof(shutdown));
}

struct nrf51vbatResponse
{
  uint8_t dummy1;
  uint8_t dummy2;
  uint8_t dummy3;
  float vbat;
} __attribute__((packed));

float Crazyflie::vbat()
{
  const uint8_t shutdown[] = {0xFF, 0xFE, 0x04};
  startBatchRequest();
  addRequest(shutdown, 2);
  handleRequests();
  return getRequestResult<nrf51vbatResponse>(0)->vbat;
}

void Crazyflie::writeFlash(
  BootloaderTarget target,
  const std::vector<uint8_t>& data)
{
  // Get info about the target
  bootloaderGetInfoRequest req(target);
  startBatchRequest();
  addRequest(req, 3);
  handleRequests(/*crtpMode=*/false);
  const bootloaderGetInfoResponse* response = getRequestResult<bootloaderGetInfoResponse>(0);
  uint16_t pageSize = response->pageSize;
  uint16_t flashStart = response->flashStart;
  uint16_t nBuffPage = response->nBuffPage;
  // std::cout << "pageSize: " << pageSize
  //           << " nBuffPage: " << nBuffPage
  //           << " nFlashPage: " << response->nFlashPage
  //           << " flashStart: " << flashStart
  //           << " version: " << (int)response->version
  //           << std::endl;

  uint16_t numPages = ceil(data.size() / (float)pageSize);
  if (numPages + flashStart >= response->nFlashPage) {
    std::stringstream sstr;
    sstr << "Requested size too large!";
    throw std::runtime_error(sstr.str());
  }
  // std::cout << "numPages: " << numPages << std::endl;

  // write flash
  size_t offset = 0;
  uint16_t usedBuffers = 0;
  // startBatchRequest();
  for (uint16_t page = flashStart; page < numPages + flashStart; ++page) {
    for (uint16_t address = 0; address < pageSize; address += 25) {
      // std::cout << "request: " << page << " " << address << std::endl;
      bootloaderLoadBufferRequest req(target, usedBuffers, address);
      size_t requestedSize = std::min<size_t>(data.size() - offset, std::min<size_t>(25, pageSize - address));
      memcpy(req.data, &data[offset], requestedSize);
      // addRequest(req, 0);
      // for (size_t i = 0; i < 10; ++i)
      // std::cout << "request: " << req.page << " " << req.address << " " << requestedSize << std::endl;
      // for (size_t i = 0; i < 10; ++i) {

      auto start = std::chrono::system_clock::now();
      // while (true) {
        sendPacketOrTimeout((uint8_t*)&req, 7 + requestedSize);
      //   startBatchRequest();
      //   bootloaderReadBufferRequest req2(target, usedBuffers, address);
      //   addRequest(req2, 7);
      //   handleRequests(/*crtpMode=*/false);
      //   const bootloaderReadBufferResponse* response = getRequestResult<bootloaderReadBufferResponse>(0);
      //   if (memcmp(req.data, response->data, requestedSize) == 0) {
      //     break;
      //   }
      //   auto end = std::chrono::system_clock::now();
      //   std::chrono::duration<double> elapsedSeconds = end-start;
      //   if (elapsedSeconds.count() > 1.0) {
      //     throw std::runtime_error("timeout");
      //   }
      // }
      offset += requestedSize;
      if (offset >= data.size()) {
        break;
      }
    }
    ++usedBuffers;
    if (usedBuffers == nBuffPage
        || page == numPages + flashStart - 1) {


      // startBatchRequest();
      // for (uint16_t buf = 0; buf < usedBuffers; ++buf) {
      //   for (uint16_t address = 0; address < pageSize; address += 25) {
      //     // std::cout << "request: " << page << " " << address << std::endl;
      //     bootloaderReadBufferRequest req(target, buf, address);
      //     addRequest(req, 7);
      //   }
      // }
      // handleRequests(/*crtpMode=*/false);


      // upload all the buffers now
      // std::cout << "try to upload buffers!" << std::endl;
      // handleRequests(/*crtpMode=*/false);
      // std::cout << "buffers uploaded!" << std::endl;

      // std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write flash
      bootloaderWriteFlashRequest req(target, 0, page - usedBuffers + 1, usedBuffers);
      sendPacketOrTimeout((uint8_t*)&req, sizeof(req));

      auto start = std::chrono::system_clock::now();

      size_t tries = 0;
      while (true) {
        Crazyradio::Ack ack;
        bootloaderFlashStatusRequest statReq(target);
        sendPacket((const uint8_t*)&statReq, sizeof(statReq), ack);
        if (   ack.ack
            && ack.size == 5
            && memcmp(&req, ack.data, 3) == 0) {
          if (ack.data[3] != 1 || ack.data[4] != 0) {
            throw std::runtime_error("Error during flashing!");
          }
          break;
        }

        // std::cout << page - usedBuffers + 1 << "," << usedBuffers << std::endl;
        // startBatchRequest();
        // bootloaderWriteFlashRequest req(target, 0, page - usedBuffers + 1, usedBuffers);
        // addRequest(req, 3);
        // handleRequests(/*crtpMode=*/false);
        // const bootloaderWriteFlashResponse* response = getRequestResult<bootloaderWriteFlashResponse>(0);
        // if (response->done == 1 && response->error == 0) {
        //   break;
        // }
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end-start;
        if (elapsedSeconds.count() > 0.5) {
          start = end;
          sendPacketOrTimeout((uint8_t*)&req, sizeof(req));
          ++tries;
          if (tries > 5) {
            throw std::runtime_error("timeout");
          }
        }
      }

      // std::cout << "Flashed: " << (page - flashStart) / (float)numPages * 100.0 << " %" << std::endl;

      // get ready to fill more buffers
      // if (page != numPages + flashStart - 1) {
      //   startBatchRequest();
      // }
      usedBuffers = 0;
    }
  }


}

void Crazyflie::readFlash(
  BootloaderTarget target,
  size_t size,
  std::vector<uint8_t>& data)
{
  // Get info about the target
  bootloaderGetInfoRequest req(target);
  startBatchRequest();
  addRequest(req, 3);
  handleRequests(/*crtpMode=*/false);
  const bootloaderGetInfoResponse* response = getRequestResult<bootloaderGetInfoResponse>(0);
  uint16_t pageSize = response->pageSize;
  uint16_t flashStart = response->flashStart;
  // std::cout << "pageSize: " << pageSize
  //           << " nBuffPage: " << response->nBuffPage
  //           << " nFlashPage: " << response->nFlashPage
  //           << " flashStart: " << flashStart
  //           << " version: " << (int)response->version
  //           << std::endl;

  uint16_t numPages = ceil(size / (float)pageSize);
  if (numPages + flashStart >= response->nFlashPage) {
    std::stringstream sstr;
    sstr << "Requested size too large!";
    throw std::runtime_error(sstr.str());
  }
  // std::cout << "numPages: " << numPages << std::endl;

  // read flash
  size_t offset = 0;
  startBatchRequest();
  for (uint16_t page = flashStart; page < numPages + flashStart; ++page) {
    for (uint16_t address = 0; address < pageSize; address += 25) {
      // std::cout << "request: " << page << " " << address << std::endl;
      bootloaderReadFlashRequest req(target, page, address);
      addRequest(req, 7);
      size_t requestedSize = std::min(25, pageSize - address);
      offset += requestedSize;
      if (offset > size) {
        break;
      }
    }
  }
  handleRequests(/*crtpMode=*/false);

  // update output
  data.resize(size);
  size_t i = 0;
  offset = 0;
  for (uint16_t page = flashStart; page < numPages + flashStart; ++page) {
    for (uint16_t address = 0; address < pageSize; address += 25) {
      const bootloaderReadFlashResponse* response = getRequestResult<bootloaderReadFlashResponse>(i++);
      size_t requestedSize = std::min(25, pageSize - address);
      // std::cout << "offset: " << offset << " reqS: " << requestedSize;
      memcpy(&data[offset], response->data, std::min(size - offset, requestedSize));
      offset += requestedSize;
      if (offset > size) {
        break;
      }
    }
  }
}

void Crazyflie::setChannel(uint8_t channel)
{
  const uint8_t setChannel[] = {0xFF, 0x03, 0x01, channel};
  sendPacketOrTimeout(setChannel, sizeof(setChannel));
}

void Crazyflie::requestLogToc(bool forceNoCache)
{
  // Find the number of log variables in TOC
  crtpLogGetInfoRequest infoRequest;
  startBatchRequest();
  addRequest(infoRequest, 1);
  handleRequests();
  size_t len = getRequestResult<crtpLogGetInfoResponse>(0)->log_len;
  uint32_t crc = getRequestResult<crtpLogGetInfoResponse>(0)->log_crc;

  // check if it is in the cache
  std::string fileName = "log" + std::to_string(crc) + ".json";
  std::ifstream infile(fileName);

  if (forceNoCache || !infile.good()) {
    m_logger.info("Log: " + std::to_string(len));

    // Request detailed information
    startBatchRequest();
    for (size_t i = 0; i < len; ++i) {
      crtpLogGetItemRequest itemRequest(i);
      addRequest(itemRequest, 2);
    }
    handleRequests();

    // Update internal structure with obtained data
    m_logTocEntries.resize(len);
    for (size_t i = 0; i < len; ++i) {
      auto response = getRequestResult<crtpLogGetItemResponse>(i);
      LogTocEntry& entry = m_logTocEntries[i];
      entry.id = i;
      entry.type = (LogType)response->type;
      entry.group = std::string(&response->text[0]);
      entry.name = std::string(&response->text[entry.group.size() + 1]);
    }

    // Write a cache file
    {
      pt::ptree root;
      pt::ptree entriesNode;
      for (const auto& entry : m_logTocEntries) {
        pt::ptree entryNode;
        entryNode.put("id", entry.id);
        entryNode.put("type", entry.type);
        entryNode.put("group", entry.group);
        entryNode.put("name", entry.name);
        // entriesNode.put("", entryNode);
        entriesNode.push_back(std::make_pair("", entryNode));
      }
      root.add_child("entries", entriesNode);
      std::ofstream output(fileName);
      write_json(output, root);
    }
  } else {
    m_logger.info("Found variables in cache.");

    pt::ptree root;
    pt::read_json(fileName, root);
    m_logTocEntries.clear();
    for (const auto& item : root.get_child("entries")) {
      m_logTocEntries.resize(m_logTocEntries.size() + 1);
      m_logTocEntries.back().id = item.second.get<uint8_t>("id");
      m_logTocEntries.back().type = (LogType)item.second.get<int>("type");
      m_logTocEntries.back().group = item.second.get<std::string>("group");
      m_logTocEntries.back().name = item.second.get<std::string>("name");
    }
  }
}

void Crazyflie::requestParamToc(bool forceNoCache)
{
  // Find the number of parameters in TOC
  crtpParamTocGetInfoRequest infoRequest;
  startBatchRequest();
  addRequest(infoRequest, 1);
  handleRequests();
  size_t len = getRequestResult<crtpParamTocGetInfoResponse>(0)->numParam;
  uint32_t crc = getRequestResult<crtpParamTocGetInfoResponse>(0)->crc;

  // check if it is in the cache
  std::string fileName = "params" + std::to_string(crc) + ".json";
  std::ifstream infile(fileName);

  if (forceNoCache || !infile.good()) {
    m_logger.info("Params: " + std::to_string(len));

    // Request detailed information and values
    startBatchRequest();
    for (size_t i = 0; i < len; ++i) {
      crtpParamTocGetItemRequest itemRequest(i);
      addRequest(itemRequest, 2);
      crtpParamReadRequest readRequest(i);
      addRequest(readRequest, 1);
    }
    handleRequests();

    // Update internal structure with obtained data
    m_paramTocEntries.resize(len);
    for (size_t i = 0; i < len; ++i) {
      auto r = getRequestResult<crtpParamTocGetItemResponse>(i*2+0);
      auto val = getRequestResult<crtpParamValueResponse>(i*2+1);

      ParamTocEntry& entry = m_paramTocEntries[i];
      entry.id = i;
      entry.type = (ParamType)(r->length | r-> type << 2 | r->sign << 3);
      entry.readonly = r->readonly;
      entry.group = std::string(&r->text[0]);
      entry.name = std::string(&r->text[entry.group.size() + 1]);

      ParamValue v;
      std::memcpy(&v, &val->valueFloat, 4);
      m_paramValues[i] = v;
    }

    // Write a cache file
    {
      pt::ptree root;
      pt::ptree entriesNode;
      for (const auto& entry : m_paramTocEntries) {
        pt::ptree entryNode;
        entryNode.put("id", entry.id);
        entryNode.put("type", entry.type);
        entryNode.put("readonly", entry.readonly);
        entryNode.put("group", entry.group);
        entryNode.put("name", entry.name);
        // entriesNode.put("", entryNode);
        entriesNode.push_back(std::make_pair("", entryNode));
      }
      root.add_child("entries", entriesNode);
      std::ofstream output(fileName);
      write_json(output, root);
    }
  } else {
    m_logger.info("Found variables in cache.");

    pt::ptree root;
    pt::read_json(fileName, root);
    m_paramTocEntries.clear();
    for (const auto& item : root.get_child("entries")) {
      m_paramTocEntries.resize(m_paramTocEntries.size() + 1);
      m_paramTocEntries.back().id = item.second.get<uint8_t>("id");
      m_paramTocEntries.back().type = (ParamType)item.second.get<int>("type");
      m_paramTocEntries.back().readonly = item.second.get<bool>("readonly");
      m_paramTocEntries.back().group = item.second.get<std::string>("group");
      m_paramTocEntries.back().name = item.second.get<std::string>("name");
    }

    // Request values
    startBatchRequest();
    for (size_t i = 0; i < len; ++i) {
      crtpParamReadRequest readRequest(i);
      addRequest(readRequest, 1);
    }
    handleRequests();
    for (size_t i = 0; i < len; ++i) {
      auto val = getRequestResult<crtpParamValueResponse>(i);
      ParamValue v;
      std::memcpy(&v, &val->valueFloat, 4);
      m_paramValues[i] = v;
    }
  }
}

void Crazyflie::startSetParamRequest()
{
  startBatchRequest();
}

void Crazyflie::addSetParam(uint8_t id, const ParamValue& value) {

  // startBatchRequest();
  bool found = false;
  for (auto&& entry : m_paramTocEntries) {
    if (entry.id == id) {
      found = true;
      switch (entry.type) {
        case ParamTypeUint8:
          {
            crtpParamWriteRequest<uint8_t> request(id, value.valueUint8);
            addRequest(request, 1);
            break;
          }
        case ParamTypeInt8:
          {
            crtpParamWriteRequest<int8_t> request(id, value.valueInt8);
            addRequest(request, 1);
            break;
          }
        case ParamTypeUint16:
          {
            crtpParamWriteRequest<uint16_t> request(id, value.valueUint16);
            addRequest(request, 1);
            break;
          }
        case ParamTypeInt16:
          {
            crtpParamWriteRequest<int16_t> request(id, value.valueInt16);
            addRequest(request, 1);
            break;
          }
        case ParamTypeUint32:
          {
            crtpParamWriteRequest<uint32_t> request(id, value.valueUint32);
            addRequest(request, 1);
            break;
          }
        case ParamTypeInt32:
          {
            crtpParamWriteRequest<int32_t> request(id, value.valueInt32);
            addRequest(request, 1);
            break;
          }
        case ParamTypeFloat:
          {
            crtpParamWriteRequest<float> request(id, value.valueFloat);
            addRequest(request, 1);
            break;
          }
      }
    }
  }

  if (!found) {
    std::stringstream sstr;
    sstr << "Could not find parameter with id " << id;
    throw std::runtime_error(sstr.str());
  }
  // handleRequests();

  m_paramValues[id] = value;
}

void Crazyflie::setRequestedParams()
{
  handleRequests();
}

void Crazyflie::setParam(uint8_t id, const ParamValue& value) {
  startBatchRequest();
  addSetParam(id, value);
  setRequestedParams();
}



void Crazyflie::trajectoryReset()
{
  crtpTrajectoryResetRequest request;
  startBatchRequest();
  addRequest(request, 1);
  handleRequests();
  m_lastTrajectoryId = 0;
}

void Crazyflie::trajectoryAdd(
    float duration,
    std::vector<float> poly_x,
    std::vector<float> poly_y,
    std::vector<float> poly_z,
    std::vector<float> poly_yaw)
{
  crtpTrajectoryAddRequest request;
  request.data.id = m_lastTrajectoryId;

  startBatchRequest();

  // Part 1
  request.data.offset = 0;
  request.data.size = 6;
  request.data.values[0] = duration;
  request.data.values[1] = poly_x[0];
  request.data.values[2] = poly_x[1];
  request.data.values[3] = poly_x[2];
  request.data.values[4] = poly_x[3];
  request.data.values[5] = poly_x[4];
  addRequest(request, 3);

  // Part 2
  request.data.offset = 6;
  request.data.size = 6;
  request.data.values[0] = poly_x[5];
  request.data.values[1] = poly_x[6];
  request.data.values[2] = poly_x[7];
  request.data.values[3] = poly_y[0];
  request.data.values[4] = poly_y[1];
  request.data.values[5] = poly_y[2];
  addRequest(request, 3);

  // Part 3
  request.data.offset = 12;
  request.data.size = 6;
  request.data.values[0] = poly_y[3];
  request.data.values[1] = poly_y[4];
  request.data.values[2] = poly_y[5];
  request.data.values[3] = poly_y[6];
  request.data.values[4] = poly_y[7];
  request.data.values[5] = poly_z[0];
  addRequest(request, 3);

  // Part 4
  request.data.offset = 18;
  request.data.size = 6;
  request.data.values[0] = poly_z[1];
  request.data.values[1] = poly_z[2];
  request.data.values[2] = poly_z[3];
  request.data.values[3] = poly_z[4];
  request.data.values[4] = poly_z[5];
  request.data.values[5] = poly_z[6];
  addRequest(request, 3);

  // Part 5
  request.data.offset = 24;
  request.data.size = 6;
  request.data.values[0] = poly_z[7];
  request.data.values[1] = poly_yaw[0];
  request.data.values[2] = poly_yaw[1];
  request.data.values[3] = poly_yaw[2];
  request.data.values[4] = poly_yaw[3];
  request.data.values[5] = poly_yaw[4];
  addRequest(request, 3);

  // Part 6
  request.data.offset = 30;
  request.data.size = 3;
  request.data.values[0] = poly_yaw[5];
  request.data.values[1] = poly_yaw[6];
  request.data.values[2] = poly_yaw[7];
  addRequest(request, 3);

  handleRequests(5.0, 0.05, 17);

  ++m_lastTrajectoryId;
}

void Crazyflie::trajectoryHover(
    float x,
    float y,
    float z,
    float yaw,
    float duration)
{
  crtpTrajectoryHoverRequest request(x, y, z, yaw, duration);
  sendPacketOrTimeout(reinterpret_cast<const uint8_t*>(&request), sizeof(request));
  // startBatchRequest();
  // addRequest(request, 2);
  // handleRequests();
}

// void Crazyflie::trajectoryStart()
// {
//   m_lastTrajectoryResponse = -1;
//   do {
//     crtpTrajectoryStartRequest request;
//     sendPacket((const uint8_t*)&request, sizeof(request));
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//   } while (m_lastTrajectoryResponse != 2);
// }

// void Crazyflie::setTrajectoryState(bool state)
// {
//   m_lastTrajectoryResponse = -1;
//   do {
//     crtpTrajectoryStateRequest request(state);
//     sendPacket((const uint8_t*)&request, sizeof(request));
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//   } while (m_lastTrajectoryResponse != 3 || m_lastTrajectoryResponse2 != state);
// }

void Crazyflie::sendPositionExternalBringup(
  const stateExternalBringup& data)
{
  crtpPosExtBringup request;
  request.data.pose[0].id = data.id;
  request.data.pose[0].x = position_float_to_fix24(data.x);
  request.data.pose[0].y = position_float_to_fix24(data.y);
  request.data.pose[0].z = position_float_to_fix24(data.z);
  float q[4] = { data.q0, data.q1, data.q2, data.q3 };
  request.data.pose[0].quat = quatcompress(q);
  request.data.pose[1].id = 0;
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void Crazyflie::setEllipse(
  const vec3& center,
  const vec3& major,
  const vec3& minor,
  float period)
{
  crtpTrajectorySetEllipseRequest request;
  request.data.centerx = position_float_to_fix16(center.x);
  request.data.centery = position_float_to_fix16(center.y);
  request.data.centerz = position_float_to_fix16(center.z);
  request.data.majorx = position_float_to_fix16(major.x);
  request.data.majory = position_float_to_fix16(major.y);
  request.data.majorz = position_float_to_fix16(major.z);
  request.data.minorx = position_float_to_fix16(minor.x);
  request.data.minory = position_float_to_fix16(minor.y);
  request.data.minorz = position_float_to_fix16(minor.z);
  request.data.period = period;

  startBatchRequest();
  addRequest(request, 2);
  handleRequests();
}

void Crazyflie::takeoff(
  uint8_t group,
  float targetHeight,
  uint16_t time_in_ms)
{
  crtpTrajectoryTakeoffRequest request(group, targetHeight, time_in_ms);

  startBatchRequest();
  addRequest(request, 2);
  handleRequests();
}

void Crazyflie::land(
  uint8_t group,
  float targetHeight,
  uint16_t time_in_ms)
{
  crtpTrajectoryLandRequest request(group, targetHeight, time_in_ms);

  startBatchRequest();
  addRequest(request, 2);
  handleRequests();
}

void Crazyflie::setGroup(
  uint8_t group)
{
  crtpTrajectorySetGroupRequest request(group);

  startBatchRequest();
  addRequest(request, 2);
  handleRequests();
}

bool Crazyflie::sendPacket(
  const uint8_t* data,
  uint32_t length)
{
  Crazyradio::Ack ack;
  sendPacket(data, length, ack);
  return ack.ack;
}

 void Crazyflie::sendPacketOrTimeout(
   const uint8_t* data,
   uint32_t length,
   float timeout)
{
  auto start = std::chrono::system_clock::now();
  while (!sendPacket(data, length)) {
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    if (elapsedSeconds.count() > timeout) {
      throw std::runtime_error("timeout");
    }
  }
}

void Crazyflie::sendPacket(
  const uint8_t* data,
  uint32_t length,
  Crazyradio::Ack& ack)
{
  static uint32_t numPackets = 0;
  static uint32_t numAcks = 0;

  numPackets++;

  if (m_radio) {
    std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
    if (m_radio->getAddress() != m_address) {
      m_radio->setAddress(m_address);
    }
    if (m_radio->getChannel() != m_channel) {
      m_radio->setChannel(m_channel);
    }
    if (m_radio->getDatarate() != m_datarate) {
      m_radio->setDatarate(m_datarate);
    }
    if (!m_radio->getAckEnable()) {
      m_radio->setAckEnable(true);
    }
    m_radio->sendPacket(data, length, ack);
  } else {
    std::unique_lock<std::mutex> mlock(g_crazyflieusbMutex[m_devId]);
    m_transport->sendPacket(data, length, ack);
  }
  ack.data[ack.size] = 0;
  if (ack.ack) {
    handleAck(ack);
    numAcks++;
  }
  if (numPackets == 100) {
    if (m_linkQualityCallback) {
      // We just take the ratio of sent vs. acked packets here
      // for a sliding window of 100 packets
      float linkQuality = numAcks / (float)numPackets;
      m_linkQualityCallback(linkQuality);
    }
    numPackets = 0;
    numAcks = 0;
  }
}

void Crazyflie::handleAck(
  const Crazyradio::Ack& result)
{
  if (crtpConsoleResponse::match(result)) {
    if (result.size > 0) {
      crtpConsoleResponse* r = (crtpConsoleResponse*)result.data;
      if (m_consoleCallback) {
        m_consoleCallback(r->text);
      }
    }
      // std::cout << r->text << std::endl;
    // }
    // ROS_INFO("Console: %s", r->text);
  }
  else if (crtpLogGetInfoResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpLogGetItemResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpLogControlResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpLogDataResponse::match(result)) {
    crtpLogDataResponse* r = (crtpLogDataResponse*)result.data;
    auto iter = m_logBlockCb.find(r->blockId);
    if (iter != m_logBlockCb.end()) {
      iter->second(r, result.size - 5);
    }
    else {
      m_logger.warning("Received unrequested data for block: " + std::to_string((int)r->blockId));
    }
  }
  else if (crtpParamTocGetInfoResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpParamTocGetItemResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpParamValueResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpPlatformRSSIAck::match(result)) {
    crtpPlatformRSSIAck* r = (crtpPlatformRSSIAck*)result.data;
    if (m_emptyAckCallback) {
      m_emptyAckCallback(r);
    }
  }
  else if (crtpTrajectoryResponse::match(result)) {
    // handled in batch system
  }
  else {
    crtp* header = (crtp*)result.data;
    m_logger.warning("Don't know ack: Port: " + std::to_string((int)header->port)
      + " Channel: " + std::to_string((int)header->channel)
      + " Len: " + std::to_string((int)result.size));
    // for (size_t i = 1; i < result.size; ++i) {
    //   std::cout << "    " << (int)result.data[i] << std::endl;
    // }
  }
}

const Crazyflie::LogTocEntry* Crazyflie::getLogTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_logTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

const Crazyflie::ParamTocEntry* Crazyflie::getParamTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_paramTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

uint8_t Crazyflie::registerLogBlock(
  std::function<void(crtpLogDataResponse*, uint8_t)> cb)
{
  for (uint8_t id = 0; id < 255; ++id) {
    if (m_logBlockCb.find(id) == m_logBlockCb.end()) {
      m_logBlockCb[id] = cb;
      return id;
    }
  }
}

bool Crazyflie::unregisterLogBlock(
  uint8_t id)
{
  m_logBlockCb.erase(m_logBlockCb.find(id));
}

// Batch system

void Crazyflie::startBatchRequest()
{
  m_batchRequests.clear();
}

void Crazyflie::addRequest(
  const uint8_t* data,
  size_t numBytes,
  size_t numBytesToMatch)
{
  m_batchRequests.resize(m_batchRequests.size() + 1);
  m_batchRequests.back().request.resize(numBytes);
  memcpy(m_batchRequests.back().request.data(), data, numBytes);
  m_batchRequests.back().numBytesToMatch = numBytesToMatch;
  m_batchRequests.back().finished = false;
}

void Crazyflie::handleRequests(
  bool crtpMode,
  float baseTime,
  float timePerRequest,
  int additionalSleep)
{
  auto start = std::chrono::system_clock::now();
  Crazyradio::Ack ack;
  m_numRequestsFinished = 0;
  bool sendPing = false;

  float timeout = baseTime + timePerRequest * m_batchRequests.size();

  while (true) {
    if (additionalSleep) {
      std::this_thread::sleep_for(std::chrono::milliseconds(additionalSleep));
    }
    if (!crtpMode || !sendPing) {
      for (const auto& request : m_batchRequests) {
        if (!request.finished) {
          // std::cout << "sendReq" << std::endl;
          sendPacket(request.request.data(), request.request.size(), ack);
          handleBatchAck(ack, crtpMode);

          auto end = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedSeconds = end-start;
          if (elapsedSeconds.count() > timeout) {
            throw std::runtime_error("timeout");
          }
        }
      }
      sendPing = true;
    } else {
      for (size_t i = 0; i < 10; ++i) {
        uint8_t ping = 0xFF;
        sendPacket(&ping, sizeof(ping), ack);
        handleBatchAck(ack, crtpMode);
        // if (ack.ack && crtpPlatformRSSIAck::match(ack)) {
        //   sendPing = false;
        // }

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end-start;
        if (elapsedSeconds.count() > timeout) {
          throw std::runtime_error("timeout");
        }
      }

      sendPing = false;
    }
    if (m_numRequestsFinished == m_batchRequests.size()) {
      break;
    }
  }
}

void Crazyflie::handleBatchAck(
  const Crazyradio::Ack& ack,
  bool crtpMode)
{
  if (ack.ack) {
    // if (ack.size > 0) {
    //   for (size_t i = 0; i < ack.size; ++i) {
    //     std::cout << (int)ack.data[i] << ",";
    //   }
    //   std::cout << std::endl;
    // }

    for (auto& request : m_batchRequests) {
      if (crtpMode) {
        if ((crtp(ack.data[0]) == crtp(request.request[0]) || ack.data[0] == request.request[0])
            && memcmp(&ack.data[1], &request.request[1], request.numBytesToMatch) == 0
            && !request.finished) {
          request.ack = ack;
          request.finished = true;
          ++m_numRequestsFinished;
          // std::cout << "gotack" <<std::endl;
          return;
        }
      } else {
        if (!request.finished
            && memcmp(&ack.data[0], &request.request[0], request.numBytesToMatch) == 0) {
          request.ack = ack;
          request.finished = true;
          ++m_numRequestsFinished;
          // std::cout << m_numRequestsFinished / (float)m_batchRequests.size() * 100.0 << " %" << std::endl;
          return;
        }
      }
    }

    // handle generic ack
    // handleAck(ack);
    // crtp c(ack.data[0]);
    //std::cout << "didnt handle ack " << (int) c.port << " " << (int) c.channel << " " << (int) ack.data[1] << " " << (int) ack.data[2] << std::endl;
    // TODO: generic handle ack here?
  }
}


////////////////////////////////////////////////////////////////

CrazyflieBroadcaster::CrazyflieBroadcaster(
  const std::string& link_uri)
  : m_radio(NULL)
  , m_devId(0)
  , m_channel(0)
  , m_address(0)
  , m_datarate(Crazyradio::Datarate_250KPS)
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%lx",
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) == 5;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) == 4;
    m_address = 0xE7E7E7E7E7;
  }

  if (success)
  {
    m_channel = channel;
    if (datarate == 250 && datarateType == 'K') {
      m_datarate = Crazyradio::Datarate_250KPS;
    }
    else if (datarate == 1 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_1MPS;
    }
    else if (datarate == 2 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_2MPS;
    }

    if (m_devId >= MAX_RADIOS) {
      throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
    }

    if (!g_crazyradios[m_devId]) {
      g_crazyradios[m_devId] = new Crazyradio(m_devId);
      // g_crazyradios[m_devId]->setAckEnable(false);
      g_crazyradios[m_devId]->setAckEnable(true);
      g_crazyradios[m_devId]->setArc(0);
    }

    m_radio = g_crazyradios[m_devId];
  }
  else {
    throw std::runtime_error("Uri is not valid!");
  }
}

void CrazyflieBroadcaster::sendPacket(
  const uint8_t* data,
  uint32_t length)
{
  std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
  if (m_radio->getAddress() != m_address) {
    m_radio->setAddress(m_address);
  }
  if (m_radio->getChannel() != m_channel) {
    m_radio->setChannel(m_channel);
  }
  if (m_radio->getDatarate() != m_datarate) {
    m_radio->setDatarate(m_datarate);
  }
  if (m_radio->getAckEnable()) {
    m_radio->setAckEnable(false);
  }
  m_radio->sendPacketNoAck(data, length);
}

void CrazyflieBroadcaster::send2Packets(
  const uint8_t* data,
  uint32_t length)
{
  std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
  if (m_radio->getAddress() != m_address) {
    m_radio->setAddress(m_address);
  }
  if (m_radio->getChannel() != m_channel) {
    m_radio->setChannel(m_channel);
  }
  if (m_radio->getDatarate() != m_datarate) {
    m_radio->setDatarate(m_datarate);
  }
  if (m_radio->getAckEnable()) {
    m_radio->setAckEnable(false);
  }
  m_radio->send2PacketsNoAck(data, length);
}

void CrazyflieBroadcaster::trajectoryStart(
  uint8_t group,
  bool reversed)
{
  crtpTrajectoryStartRequest request(group, reversed);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

// void CrazyflieBroadcaster::setTrajectoryState(bool state)
// {
//   crtpTrajectoryStateRequest request(state);
//   sendPacket((const uint8_t*)&request, sizeof(request));
// }

void CrazyflieBroadcaster::takeoff(
  uint8_t group,
  float targetHeight,
  uint16_t time_in_ms)
{
  // crtpTrajectoryTakeoffRequest request(1.0, 2000);
  crtpTrajectoryTakeoffRequest request(group, targetHeight, time_in_ms);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void CrazyflieBroadcaster::land(
  uint8_t group,
  float targetHeight,
  uint16_t time_in_ms)
{
  // crtpTrajectoryLandRequest request(0.06, 2000);
  crtpTrajectoryLandRequest request(group, targetHeight, time_in_ms);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void CrazyflieBroadcaster::ellipse(
  uint8_t group)
{
  crtpTrajectoryStartEllipseRequest request(group);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void CrazyflieBroadcaster::goHome(
  uint8_t group)
{
  crtpTrajectoryGoHomeRequest request(group);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void CrazyflieBroadcaster::startCannedTrajectory(
  uint8_t group,
  uint16_t trajectory, // one of enum trajectory_type
  float timescale)
{
  crtpTrajectoryStartCannedRequest request(group, trajectory, timescale);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void CrazyflieBroadcaster::sendPositionExternalBringup(
  const std::vector<stateExternalBringup>& data)
{
  if (data.size() == 0) {
    return;
  }

  std::vector<crtpPosExtBringup> requests(ceil(data.size() / 2.0));
  for (size_t i = 0; i < data.size(); ++i) {
    size_t j = i / 2;
    requests[j].data.pose[i%2].id = data[i].id;
    requests[j].data.pose[i%2].x = position_float_to_fix24(data[i].x);
    requests[j].data.pose[i%2].y = position_float_to_fix24(data[i].y);
    requests[j].data.pose[i%2].z = position_float_to_fix24(data[i].z);
    float q[4] = { data[i].q0, data[i].q1, data[i].q2, data[i].q3 };
    requests[j].data.pose[i%2].quat = quatcompress(q);
  }

  size_t remainingRequests = requests.size();
  size_t i = 0;
  while (remainingRequests > 0) {
    if (remainingRequests >= 2) {
      send2Packets(reinterpret_cast<const uint8_t*>(&requests[i]), 2 * sizeof(crtpPosExtBringup));
      remainingRequests -= 2;
      i += 2;
    } else {
      sendPacket(reinterpret_cast<const uint8_t*>(&requests[i]), sizeof(crtpPosExtBringup));
      remainingRequests -= 1;
      i += 1;
    }
  }
}

void CrazyflieBroadcaster::sendPacketDropTest(
    uint64_t seq)
{
  crtpPacketDropTest request(seq);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void CrazyflieBroadcaster::setParam(
  uint8_t group,
  uint8_t id,
  Crazyflie::ParamType type,
  const Crazyflie::ParamValue& value) {

  switch (type) {
    case Crazyflie::ParamTypeUint8:
      {
        crtpParamWriteBroadcastRequest<uint8_t> request(group, id, value.valueUint8);
        sendPacket((const uint8_t*)&request, sizeof(request));
        break;
      }
    case Crazyflie::ParamTypeInt8:
      {
        crtpParamWriteBroadcastRequest<int8_t> request(group, id, value.valueInt8);
        sendPacket((const uint8_t*)&request, sizeof(request));
        break;
      }
    case Crazyflie::ParamTypeUint16:
      {
        crtpParamWriteBroadcastRequest<uint16_t> request(group, id, value.valueUint16);
        sendPacket((const uint8_t*)&request, sizeof(request));
        break;
      }
    case Crazyflie::ParamTypeInt16:
      {
        crtpParamWriteBroadcastRequest<int16_t> request(group, id, value.valueInt16);
        sendPacket((const uint8_t*)&request, sizeof(request));
        break;
      }
    case Crazyflie::ParamTypeUint32:
      {
        crtpParamWriteBroadcastRequest<uint32_t> request(group, id, value.valueUint32);
        sendPacket((const uint8_t*)&request, sizeof(request));
        break;
      }
    case Crazyflie::ParamTypeInt32:
      {
        crtpParamWriteBroadcastRequest<int32_t> request(group, id, value.valueInt32);
        sendPacket((const uint8_t*)&request, sizeof(request));
        break;
      }
    case Crazyflie::ParamTypeFloat:
      {
        crtpParamWriteBroadcastRequest<float> request(group, id, value.valueFloat);
        sendPacket((const uint8_t*)&request, sizeof(request));
        break;
      }
  }
  // m_paramValues[id] = value;
}
