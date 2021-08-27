


#include <iostream>
#include <string>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cmath>

#include <vector>
#include <functional>
#include <filesystem>

#include <nlohmann/json.hpp>
#include <phosphor-logging/log.hpp>
#include <phosphor-logging/elog-errors.hpp>

#include "pid.hpp"
#include "util.hpp"
#include "fan.h"

#include "zone.hpp"


namespace fs = std::filesystem;
using namespace std::literals::chrono_literals;
using json = nlohmann::json;

// #define RIKFAN_DEBUG


SensorFS::SensorFS(const std::string &p, double err_val) : error_value{err_val}, state{0}
{
    auto pos = p.find('*');
    if (pos == std::string::npos)
    {
        real_path = p;
        state = 10;
        return;
    }

    auto ppos = p.rfind('/', pos);
    auto fpos = p.find('/', pos);
    try
    {
        for (const auto &dir : fs::directory_iterator(p.substr(0, ppos) + "/hwmon/"))
        {
            real_path = (dir.path() / p.substr(++fpos)).string();
            state = 10;
            break;
        }
    }
    catch (std::exception &e)
    {
        real_path = p;
        state = 0;
    }
}

double SensorFS::get_value()
{
    std::ifstream ifs;
    double val = error_value;
    if (state)
    {
        ifs.open(real_path);
        if (ifs.is_open())
        {
            long read_val;
            ifs >> read_val;
            if (ifs.good())
            {
                val = static_cast<double>(read_val) / 1000.0;
                state = 10;
            }
            else
            {
                val = error_value;
                state--;
            }
            ifs.close();
        }
    }
    return val;
}

void SensorFS::set_value(double in)
{
    std::ofstream ofs;
    int val = static_cast<int>(std::round(in));

    ofs.open(real_path);
    if (ofs.is_open())
    {
        ofs << val;
        ofs.close();
    }
}

SensorDBus::SensorDBus(const std::string &p, double err_val, std::shared_ptr<sdbusplus::asio::connection>& b) :
    error_value{err_val},
    state{0},
    real_path{p},
    passive_bus{b}
{}

double SensorDBus::get_value()
{
    double val = error_value;
    if (state)
    {
        auto mapper = passive_bus->new_method_call(
                          "xyz.openbmc_project.CPUSensor",
                          real_path.c_str(),
                          "org.freedesktop.DBus.Properties", "Get");
        mapper.append("xyz.openbmc_project.Sensor.Value");
        mapper.append("Value");
        std::variant<double> respData;
        try
        {
            auto resp = passive_bus->call(mapper);
            resp.read(respData);
            val = std::get<double>(respData);
        }
        catch (sdbusplus::exception_t&)
        {
            val = error_value;
        }
    }
    return val;
}


Zone::Zone(std::string n,
           std::string t,
           ec::pidinfo &pidinfo_initial,
           std::vector<std::string> &s,
           std::vector<std::string> &f,
           double sp,
           long long ms,
           std::shared_ptr<sdbusplus::asio::connection>& conn_
          ) : name(n), type(t), setpt(sp)
{
    pmainthread = nullptr;
    if (ms < loop_min_delay)
        millisec = loop_min_delay;
    else
        millisec = ms;

    if (type == "one")
        error_read_temp = 0.0;
    else
        error_read_temp = margin_error_read_temp;

    for(const auto &path : s)
    {
    	if(path.substr(0, 5) == "/xyz/")
    		sensors.push_back(std::make_unique<SensorDBus>(path, error_read_temp, conn_));
    	else
    		sensors.push_back(std::make_unique<SensorFS>(path, error_read_temp));
    }

    for(const auto &path: f)
    {
        if(path.substr(0, 5) == "/xyz/")
            sensors.push_back(std::make_unique<SensorDBus>(path, error_read_temp, conn_));
        else
            sensors.push_back(std::make_unique<SensorFS>(path, error_read_temp));
    }

    initializePIDStruct(&pid_info, pidinfo_initial);
}

Zone::~Zone()
{
    stop();
}

void Zone::start()
{
    if (pmainthread != nullptr)
        stop();
    stop_flag = false;
    manualmode = false;
#ifdef RIKFAN_DEBUG
    sample_time = std::chrono::system_clock::now();
#endif
    pmainthread = std::make_unique<std::thread>(&Zone::zone_control_loop, this);
}

void Zone::stop()
{
    if (pmainthread != nullptr)
    {
        stop_flag = true;
        pmainthread->join();
        pmainthread = nullptr;
    }
}

void Zone::command(const char *cmd)
{
    if (std::strcmp(cmd, "manual") == 0)
    {
        manualmode = true;
    }
    else if (std::strcmp(cmd, "auto") == 0)
    {
        manualmode = false;
    }
    else if (std::strcmp(cmd, "on") == 0)
    {
        pid_info.lastOutput = pid_info.outLim.min;
    }
}



double Zone::processInputs()
{
    std::string actual_path;
    std::ifstream ifs;
    double retval = 0;
    for (auto &sens : sensors)
    {
        retval = std::max(retval, sens->get_value());
    }

    // for type == "one"
    if (retval == 0)
        retval = margin_error_read_temp;

    return retval;
}

double Zone::processPID(double in)
{
    return ec::pid(&pid_info, in, setpt);
}

void Zone::processOutputs(double in)
{
    for (auto &pwm : pwms)
    {
        pwm->set_value(in);
    }
}

void Zone::zone_control_loop(Zone *zone)
{
    auto delay = std::chrono::milliseconds(zone->millisec);
    while (!zone->stop_flag)
    {
        if (zone->manualmode)
        {
            std::this_thread::sleep_for(delay);
        }
        else
        {
            auto input = zone->processInputs();
            auto output = zone->processPID(input);
            zone->processOutputs(output);

#ifdef RIKFAN_DEBUG

            auto new_sample = std::chrono::system_clock::now();

            std::ofstream ofs;
            ofs.open(fs::path("/tmp/rikfan") / zone->name);
            if (ofs.is_open())
            {
                ofs << "setpoint: " << zone->setpt;
                ofs << "\ninput:    " << input;
                ofs << "\noutput:   " << output;
                std::chrono::duration<double> diff = new_sample - zone->sample_time;
                zone->sample_time = new_sample;
                ofs << "\nsample_time: " << diff.count();
                ofs << "\n\n";
                dumpPIDStruct(ofs, &zone->pid_info);
                ofs << std::endl;
                ofs.close();
            }
#endif // RIKFAN_DEBUG		

            std::this_thread::sleep_for(delay);
        }
    }
    zone->processOutputs(stop_output_const);
}




/**
 * ZoneManager
 */

ZoneManager::ZoneManager(fs::path conf_fname, std::shared_ptr<sdbusplus::asio::connection>& conn_) : conn(conn_)
{
    // fs::path conf_fname = "/etc/rikfan/conf.json";
    if (!fs::exists(conf_fname))
    {
        conf_fname = "/tmp/rikfan/conf.json";
        if (!fs::exists(conf_fname))
        {
            std::cerr << "Need config file in '/etc/rikfan/conf.json' or '/tmp/rikfan/conf.json'";
            return;
        }
    }

    std::ifstream conf_stream {conf_fname};
    json conf_json;
    try
    {
        conf_stream >> conf_json;
    }
    catch (const std::exception &e)
    {
        // std::cerr << e.what() << std::endl;
        syslog(LOG_ERR, "exception: %s", e.what());
    }

    if (conf_json.count("zones") > 0)
    {
        for (const auto &z : conf_json["zones"])
        {
            ec::pidinfo pid_conf;
            std::vector<std::string> sens_vect;
            std::vector<std::string> pwm_vect;
            double setpoint;
            std::string zone_name;
            std::string zone_type;

            z["inputs"].get_to(sens_vect);
            z["fans_pwm"].get_to(pwm_vect);
            z["name"].get_to(zone_name);
            z["type"].get_to(zone_type);
            z["setpoint"].get_to(setpoint);

            auto p = z["pid"];
            p["samplePeriod"].get_to(pid_conf.ts);
            p["proportionalCoeff"].get_to(pid_conf.proportionalCoeff);
            p["integralCoeff"].get_to(pid_conf.integralCoeff);
            p["feedFwdOffsetCoeff"].get_to(pid_conf.feedFwdOffset);
            p["feedFwdGainCoeff"].get_to(pid_conf.feedFwdGain);
            p["integralLimit_min"].get_to(pid_conf.integralLimit.min);
            p["integralLimit_max"].get_to(pid_conf.integralLimit.max);
            p["outLim_min"].get_to(pid_conf.outLim.min);
            p["outLim_max"].get_to(pid_conf.outLim.max);
            p["slewNeg"].get_to(pid_conf.slewNeg);
            p["slewPos"].get_to(pid_conf.slewPos);

            zones.emplace_back(std::make_unique<Zone>(zone_name, zone_type, pid_conf, sens_vect, pwm_vect, setpoint, pid_conf.ts * 1000, conn_));
        }
    }
}


void ZoneManager::setFanMode(unsigned int mode)
{
    if (mode > 3)
        mode = 0;   // 0 - автоматический режим

    if (mode == 0)
    {
        for (const auto &z : zones)
            z->command("auto");
    }
    else
    {
        for (const auto &z : zones)
            z->command("manual");
        // Вручную установить значения PWM
        setPWM(mode);
    }
}


void ZoneManager::start()
{
    // Запускаем циклы управления вентиляторами по зонам
    for (auto &z : zones)
        z->start();
}
