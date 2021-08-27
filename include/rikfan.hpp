
#pragma once

#include <boost/asio/io_service.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include "zone.hpp"


static constexpr const char* RikfanServiceName =
    "xyz.openbmc_project.rikfan";
static constexpr const char* RikfanIface =
    "xyz.openbmc_project.Rikfan";
static constexpr const char* RikfanPath =
    "/xyz/openbmc_project/rikfan";

class RikfanMgr
{
    enum class RikfanMode
    {
        AUTO = 0,
        MINIMAL = 1,
        OPTIMAL = 2,
        MAXIMAL = 3
    };

    boost::asio::io_service& io;
    sdbusplus::asio::object_server& server;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::shared_ptr<sdbusplus::asio::dbus_interface> iface;

    int mode = 2;

    std::unordered_map<std::string, std::string> readAllVariable();
    void setFanMode(const std::string& mode);
    int readConf();
    void writeConf(int m);

    std::unique_ptr<ZoneManager> zones;

  public:
    RikfanMgr(boost::asio::io_service& io,
                sdbusplus::asio::object_server& srv,
                std::shared_ptr<sdbusplus::asio::connection>& conn_);
};
