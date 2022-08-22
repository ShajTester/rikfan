
#include "rikfan.hpp"
#include "zone.hpp"

#include <vector>
#include <unordered_map>
#include <filesystem>

#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <boost/process/child.hpp>
#include <boost/process/io.hpp>

#include <xyz/openbmc_project/Common/error.hpp>

namespace fs = std::filesystem;


RikfanMgr::RikfanMgr(boost::asio::io_service& io_,
                     sdbusplus::asio::object_server& srv_,
                     std::shared_ptr<sdbusplus::asio::connection>& conn_) :
    io(io_), server(srv_), conn(conn_),
    zones(std::make_unique<ZoneManager>("/etc/rikfan/conf.json", io_))
{
    iface = server.add_interface(RikfanPath, RikfanIface);
    iface->register_method("ReadMode", [this]() { return this->mode; });

    iface->register_method(
        "WriteMode", [this](const std::string& mode) {
            this->setFanMode(mode);
        });

    iface->initialize(true);

    zones->start();

    this->mode = readConf();
    setFanMode(std::to_string(this->mode));
}


void RikfanMgr::setFanMode(const std::string& mode)
{
    phosphor::logging::log<phosphor::logging::level::INFO>(("Rikfan set mode " + mode).c_str());

    try 
    {
       this->mode = std::stoi(mode);
    } 
    catch (const std::exception& e) 
    { 
         // std::cout << e.what();
        this->mode = 2;
    }
    zones->setFanMode(this->mode);
    writeConf(this->mode);
    return;
}


int RikfanMgr::readConf()
{
    int m = 2;
    fs::path conf_fname = "/etc/rikfan/rikfan.conf";
    try
    {
        std::ifstream conf_stream {conf_fname};
        conf_stream >> m;
    }
    catch (const std::exception& e)
    {
        m = 2;
        writeConf(m);
    }
    return m;
}


void RikfanMgr::writeConf(int m)
{
    fs::path conf_fname = "/etc/rikfan/rikfan.conf";
    std::ofstream conf_stream {conf_fname};
    conf_stream << m;
}

