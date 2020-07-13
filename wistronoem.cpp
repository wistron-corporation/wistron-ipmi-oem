#include "config.h"

#include "wistronoem.hpp"

#include "smbus.hpp"

#include <endian.h>
#include <ipmid/api.h>
#include <stdio.h>
#include <string>
#include <systemd/sd-bus.h>

#include <fstream>
#include <functional>
#include <iostream>
#include <ipmid-host/cmd.hpp>
#include <memory>

#include <sdbusplus/bus.hpp>
#include <sdbusplus/exception.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/bus/match.hpp>

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include "i2c.h"

#define REQUEST_GPU_PRESENT 0x01
#define REQUEST_GPU_GONE 0x00

#define SWITCH_SLAVE_ADDRESS 112 /* Hexadecimal value:0x70 */
#define RISERF_SLAVE_ADDRESS 16  /* Hexadecimal value:0x10 */

void register_detect_riserf() __attribute__((constructor));
void register_detect_gpu() __attribute__((constructor));

template <typename T>
void setProperty(sdbusplus::bus::bus& bus, const std::string& busName,
                 const std::string& objPath, const std::string& interface,
                 const std::string& property, const T& value)
{
    sdbusplus::message::variant<T> data = value;

    try
    {
        auto methodCall = bus.new_method_call(
             busName.c_str(), objPath.c_str(), DBUS_PROPERTY_IFACE, "Set");

        methodCall.append(interface.c_str());
        methodCall.append(property);
        methodCall.append(data);

            auto reply = bus.call(methodCall);
    }
    catch (const std::exception& e)
    {
        /*log<level::ERR>("Set properties fail.",
                        entry("ERROR = %s", e.what()),
                        entry("Object path = %s", objPath.c_str()));*/
        return;
    }
}

bool getProperty(sdbusplus::bus::bus& bus, const std::string& busName,
                 const std::string& objPath, const std::string& interface,
                 const std::string& property)
{
    auto methodCall = bus.new_method_call(busName.c_str(), objPath.c_str(),
                                          DBUS_PROPERTY_IFACE, "Get");

    methodCall.append(interface.c_str());
    methodCall.append(property);

    //sdbusplus::message::variant<Property> value;

    try
    {
        auto reply = bus.call(methodCall);
        //reply.read(value);
    }
    catch (const std::exception& e)
    {
        /*log<level::ERR>("Get properties fail.",
                        entry("ERROR = %s", e.what()),
                        entry("Object path = %s", objPath.c_str()));*/
        return false;
    }

    //return sdbusplus::message::variant_ns::get<Property>(value);
    return true;
}

ipmi_ret_t ipmi_wistron_detect_riserf(ipmi_netfn_t netfn, ipmi_cmd_t cmd,
                                      ipmi_request_t request,
                                      ipmi_response_t response,
                                      ipmi_data_len_t data_len,
                                      ipmi_context_t context)
{
    phosphor::smbus::Smbus smbus;

    auto init_cp0 = smbus.smbusInit(9);
    if (init_cp0 == initError)
    {
        std::cerr << "smbusInit fail!"
                  << "\n";
        return false;
    }
    smbus.SetSmbusCmdByte(9, SWITCH_SLAVE_ADDRESS, 0, 8);
    auto res_cp0 = smbus.GetSmbusCmdByte(9, RISERF_SLAVE_ADDRESS, command);

    auto init_cp1 = smbus.smbusInit(10);
    if (init_cp1 == initError)
    {
        std::cerr << "smbusInit fail!"
                  << "\n";
        return false;
    }
    smbus.SetSmbusCmdByte(10, SWITCH_SLAVE_ADDRESS, 0, 8);
    auto res_cp1 = smbus.GetSmbusCmdByte(10, RISERF_SLAVE_ADDRESS, command);

    // CPU0 & CPU1 are not present.
    if (res_cp0 < 0 && res_cp1 < 0)
    {
        *data_len = sizeof(a);
        memcpy(response, &a, *data_len);
    }

    // CPU0 isn't present but CPU1 is present.
    if (res_cp0 < 0 && res_cp1 >= 0)
    {
        *data_len = sizeof(b);
        memcpy(response, &b, *data_len);
    }

    // CPU0 is present but CPU1 isn't present.
    if (res_cp0 >= 0 && res_cp1 < 0)
    {
        *data_len = sizeof(c);
        memcpy(response, &c, *data_len);
    }

    // CPU0 & CPU1 are present.
    if (res_cp0 >= 0 && res_cp1 >= 0)
    {
        *data_len = sizeof(d);
        memcpy(response, &d, *data_len);
    }

    smbus.smbusClose(9);
    smbus.smbusClose(10);

    return IPMI_CC_OK;
}

ipmi_ret_t ipmi_wistron_detect_gpu(ipmi_netfn_t netfn, ipmi_cmd_t cmd,
                                   ipmi_request_t request,
                                   ipmi_response_t response,
                                   ipmi_data_len_t data_len,
                                   ipmi_context_t context)
{
    uint8_t* req = reinterpret_cast<uint8_t*>(request);
    data_len = 0;
    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    
    if (*req == REQUEST_GPU_PRESENT) //GPUs are present 
    {    for (int i = 0; i < 8; i++)
        {
            std::string gpuPath;
            gpuPath = GPU_OBJ_PATH + std::to_string(i);
            getProperty(bus, GPU_REQUEST_NAME, gpuPath, Value_IFACE, "Value");
            if (getProperty(bus, GPU_REQUEST_NAME, gpuPath, Value_IFACE, "Value") == true)
            {
                setProperty(bus, INVENTORY_BUSNAME, INVENTORY_PATH,
                            ITEM_IFACE, "Present", true);
                return IPMI_CC_OK;
            }
            if (getProperty(bus, GPU_REQUEST_NAME, gpuPath, Value_IFACE, "Value") == false)
            {
                int present = 0;
                present |= (0x01 << i);
                if (present |= 0xff) //dbus objects of GPUs all gone
                {
                    setProperty(bus, INVENTORY_BUSNAME, INVENTORY_PATH,
                                ITEM_IFACE, "Present", false);
                    return IPMI_CC_OK;
                }
            }
        }
    }
    if (*req == REQUEST_GPU_GONE) //GPUs aren't present
    {
        for (int i = 0; i < 8; i++)
        {
            std::string gpuPath;
            gpuPath = GPU_OBJ_PATH + std::to_string(i);
            getProperty(bus, GPU_REQUEST_NAME, gpuPath, Value_IFACE, "Value");
            if (getProperty(bus, GPU_REQUEST_NAME, gpuPath, Value_IFACE, "Value") == false)
            {
                int present = 0;
                present |= (0x01 << i);
                if (present |= 0xff) //dbus object of GPUs all gone
                {
                    setProperty(bus, INVENTORY_BUSNAME, INVENTORY_PATH,
                                ITEM_IFACE, "Present", true);
                    return IPMI_CC_OK;
                }
            }
        }
    }
    return IPMI_CC_OK;
}

void register_detect_riserf()
{
    ipmi_register_callback(NETFUN_OEM, IPMI_CMD_DETECT_RISERF, NULL,
                           ipmi_wistron_detect_riserf, SYSTEM_INTERFACE);
}

void register_detect_gpu()
{
    ipmi_register_callback(NETFUN_OEM, IPMI_CMD_DETECT_GPU, NULL,
                           ipmi_wistron_detect_gpu, SYSTEM_INTERFACE);
}
