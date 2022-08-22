

#include <vector>
#include <string>

#include <boost/asio/io_service.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <phosphor-logging/log.hpp>
#include "log-conf.hpp"

#include "rikfan.hpp"
#include "zone.hpp"
#include "fan.h"


void logger_send(int loglevel, const std::string &sensor_type, const std::string &sensor_address)
{
    std::string msg;
    std::string rmid;
    if(loglevel == LOG_INFO)
    {
        rmid = std::string{redfish_message_id} + "OK";
        msg = std::string{"Sensor "} + sensor_type + " (" + sensor_address + ") OK";
    }
    else
    {
        rmid = std::string{redfish_message_id} + "Error";
        msg = std::string{"Sensor "} + sensor_type + " (" + sensor_address + ") error";
    }

    sd_journal_send("MESSAGE=%s", msg.c_str(), "PRIORITY=%i", loglevel,
                "REDFISH_MESSAGE_ID=%s", rmid.c_str(),
                "REDFISH_MESSAGE_ARGS=%s,%s", sensor_type.c_str(), sensor_address.c_str(), NULL);
}


int main()
{
    boost::asio::io_service io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name(RikfanServiceName);
    sdbusplus::asio::object_server server {conn};

    // Подготовка конфигурации вентиляторов
    auto inverted_fans = test_inverted_fans();
    // Зоны регулирования
    ZoneManager zoneManager {"/etc/rikfan/conf.json", io};
    zoneManager.setPwmInv(inverted_fans);
    // Подключение к dbus
    RikfanMgr rikfanMgr {io, server, conn, zoneManager};

    io.run();
}
