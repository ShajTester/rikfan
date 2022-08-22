

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

#include "zone.hpp"
#include "log-conf.hpp"

#include <regex>


namespace fs = std::filesystem;
using namespace std::literals::chrono_literals;
using json = nlohmann::json;


SensorFS::SensorFS(const std::string &p, double err_val) : 
        error_value{err_val},
        state{0},
        error_sensor_state{false},
        inverted{false}
{
    std::string path;

    auto pos = p.rfind("_inv");
    if(pos != std::string::npos)
    {
        path = p.substr(0, pos);
        inverted = true;
        std:: cout << "inverted " << path << std::endl;
    }
    else
    {
        path = p;
    }

    pos = path.find('*');
    if (pos == std::string::npos)
    {
        real_path = path;
        state = 10;
        return;
    }

    auto ppos = path.rfind('/', pos);
    auto fpos = path.find('/', pos);
    try
    {
        for (const auto &dir : fs::directory_iterator(path.substr(0, ppos) + "/hwmon/"))
        {
            real_path = (dir.path() / path.substr(++fpos)).string();
            state = 10;
            break;
        }
    }
    catch (std::exception &e)
    {
        real_path = path;
        state = 0;
    }

    const std::regex r{"\\d-(\\d|[a-f]){4}"};
    std::smatch m;
    try
    {
        if(std::regex_search(real_path, m, r))
            sensor_id = m[0].str();
        else
            sensor_id = real_path;
    }
    catch(const std::regex_error& e)
    {
        sensor_id = real_path;
        phosphor::logging::log<phosphor::logging::level::WARNING>((std::string{"Rikfan regex "} + e.what()).c_str());
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
                error_sensor_state = false;
            }
            else
            {
                val = error_value;
            }
            ifs.close();
        }
        else
        {
            val = error_value;
            state--;
        }
    }
    else
    {
        if(!error_sensor_state)
        {
            logger_send(LOG_WARNING, "SensorFS", sensor_id.c_str());
            error_sensor_state = true;
        }
    }
    return val;
}

void SensorFS::set_value(double in)
{
    std::ofstream ofs;
    int val = static_cast<int>(std::round(in));

    if(inverted)
        val = 256 - val;

    ofs.open(real_path);
    if (ofs.is_open())
    {
        if(inverted)
        {
            ofs << (255 - val);
        }
        else
        {
            ofs << val;
        }
        ofs.close();
    }
}

std::string SensorFS::repr()
{
    return std::string{"FileSystem: (" + std::to_string(state) + ") " + real_path};
}


SensorDBus::SensorDBus(const std::string &p, double err_val, std::shared_ptr<sdbusplus::asio::connection>& b) :
    error_value{err_val},
    state{10},
    real_path{p},
    passive_bus{b},
    inverted{false}
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
            if(!std::isnormal(val))
            {
#ifdef RIKFAN_DEBUG
                report->nan_cnt++;
#endif // RIKFAN_DEBUG            
                val = error_value;
            }
            else if(val > 100.0)
            {
#ifdef RIKFAN_DEBUG
                report->val_cnt++;
#endif // RIKFAN_DEBUG            
                val = error_value;
            }
#ifdef RIKFAN_DEBUG
            else
            {
                report->main_cnt++;
            }
#endif // RIKFAN_DEBUG            
            // phosphor::logging::log<phosphor::logging::level::INFO>(std::to_string(val).c_str());
            state = 10;
        }
        catch (sdbusplus::exception_t& e)
        {
            logger_send(LOG_WARNING, "SensorDBus", real_path.c_str());
            val = error_value;
            // state--;
        }
    }
    return val;
}

std::string SensorDBus::repr()
{
    return std::string{"DBus: (" + std::to_string(state) + ") " + real_path};
}


class CPUSensors : public SensorBase
{
    std::vector<std::unique_ptr<SensorDBus>> sensors;
    std::string real_path;
    double error_value;
    std::shared_ptr<sdbusplus::asio::connection> passive_bus;
    bool initialized;
    bool error_host_state;

#ifdef RIKFAN_DEBUG
    std::unordered_map<std::string, std::shared_ptr<DBusReport>> report;
#endif

public:
    CPUSensors(const std::string &p, double err_val, std::shared_ptr<sdbusplus::asio::connection>& b) :
        real_path(p), error_value(err_val), passive_bus(b), initialized(false), error_host_state(true)
    {
        sensors.reserve(10);
    }

    double get_value() override
    {
        double retval = 0.0;
        if (sensors.size() != 0)
        {
            for (auto &sens : sensors)
            {
                retval = std::max(retval, sens->get_value());
            }

#ifdef RIKFAN_DEBUG            
            std::ofstream ofs{"/tmp/CPUSensors"};
            for(const auto &[key, value] : report)
            {
                ofs << key << " " << value->main_cnt << " " << value->nan_cnt << " " << value->val_cnt << "\n";
            }
#endif // RIKFAN_DEBUG
            if(retval > 1.0)
            {
                // олучили правильный ответ - снимаем флаг,
                // который сбрасывает ПИД. Т.е. ПИД с этого
                // момента больше не сбрасываем.
                initialized = false;
                if(error_host_state)
                {
#ifdef RIKFAN_DEBUG
                    logger_send(LOG_INFO, "CPUSensors", real_path.c_str());
#endif // RIKFAN_DEBUG
                    error_host_state = false;
                }
            }
            else
            {
                initialized = true;
                // Получили ошибку при чтении датчиков. Ни один
                // не прочитали. М.б. выключен хост. М.б. что-то другое.
                if(!error_host_state)
                {
#ifdef RIKFAN_DEBUG
                    logger_send(LOG_WARNING, "CPUSensors", real_path.c_str());
#endif // RIKFAN_DEBUG
                    // Это чтобы не забивать логи
                    error_host_state = true;
                }
            }
        }
        else
        {
            scan_subtree();
            retval = error_value;
        }
        return retval;
    }


    void set_value(double in) override
    {
        throw "Not implemented yet.";
    }


    std::string repr() override
    {
        return std::string{"CPUSensors class"};
    }

    bool init_complete() override
    {
        return initialized;
    }

    void set_inverted(bool value) override {}
    const std::string& get_path() override {return real_path;}


private:
    void scan_subtree()
    {
        auto mapper = passive_bus->new_method_call(
                          "xyz.openbmc_project.CPUSensor",
                          real_path.c_str(),
                          "org.freedesktop.DBus.Introspectable", "Introspect");
        std::string respData;
        try
        {
            std::string needle{"node name=\""};
            auto resp = passive_bus->call(mapper);
            resp.read(respData);
            auto it = respData.begin();
            while ((it = std::search(it, respData.end(), needle.begin(), needle.end())) != respData.end())
            {
                auto ite = std::find(it + needle.size(), respData.end(), '\"');
                sensors.push_back(
                    std::make_unique<SensorDBus>(
                        real_path + "/" + std::string{it + needle.size(), ite},
                        error_value,
                        passive_bus));

#ifdef RIKFAN_DEBUG
                auto rep_blk = std::make_shared<DBusReport>();
                rep_blk->main_cnt = 0;
                rep_blk->nan_cnt = 0;
                rep_blk->val_cnt = 0;
                sensors.back()->set_report(rep_blk);
                report[std::string{it + needle.size(), ite}] = std::move(rep_blk);
#endif // RIKFAN_DEBUG            

                it = ite;
            }
            phosphor::logging::log<phosphor::logging::level::INFO>("Introspected completely");
            // Сбросить ПИД
            initialized = true;

#ifdef RIKFAN_DEBUG            
            // std::ofstream ofs{"/tmp/CPUSensors"};
            // for(const auto &sens : sensors)
            // {
            //     ofs << sens->repr() << "\n";
            // }
#endif // RIKFAN_DEBUG            
        }
        catch (sdbusplus::exception_t& e)
        {
            // phosphor::logging::log<phosphor::logging::level::ERR>("CPUSensors::scan_subtree error");
            // phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
        }
    }
};



Zone::Zone(std::string n,
           std::string t,
           ec::pidinfo &pidinfo_initial,
           std::vector<std::string> &s,   // Sensors
           std::vector<std::string> &f,   // PWMs
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

    for (const auto &path : s)
    {
        if (path.substr(0, 5) == "/xyz/")
            sensors.push_back(std::make_unique<CPUSensors>(path, error_read_temp, conn_));
        else
            sensors.push_back(std::make_unique<SensorFS>(path, error_read_temp));
    }

    for (const auto &path : f)
    {
        if (path.substr(0, 5) == "/xyz/")
            pwms.push_back(std::make_unique<SensorDBus>(path, error_read_temp, conn_));
        else
            pwms.push_back(std::make_unique<SensorFS>(path, error_read_temp));
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
    phosphor::logging::log<phosphor::logging::level::INFO>(("Zone " + name + " started").c_str());
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
        std::this_thread::sleep_for(std::chrono::milliseconds(millisec));
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
    double retval = 0.0;
    bool init_complete = false;
    for (auto &sens : sensors)
    {
        init_complete = init_complete || sens->init_complete();
        retval = std::max(retval, sens->get_value());
    }

    // for type == "one"
    if (retval < 1.0)
        retval = margin_error_read_temp;
    else if(init_complete)
    {
        pid_info.lastOutput = pid_info.outLim.min;
        phosphor::logging::log<phosphor::logging::level::INFO>(("Zone " + name + " PID reset").c_str());
    }

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
    phosphor::logging::log<phosphor::logging::level::INFO>(("Thread for zone " + zone->name + " started").c_str());
    while (!zone->stop_flag)
    {
        if (zone->manualmode)
        {
            std::this_thread::sleep_for(delay);
        }
        else
        {
#ifdef RIKFAN_DEBUG
            auto input_time_start = std::chrono::system_clock::now();
#endif // RIKFAN_DEBUG      
            auto input = zone->processInputs();
#ifdef RIKFAN_DEBUG
            auto input_time_end = std::chrono::system_clock::now();
            std::chrono::duration<double> get_input_time = input_time_end - input_time_start;
#endif // RIKFAN_DEBUG      
            auto output = zone->processPID(input);
            zone->processOutputs(output);


#ifdef RIKFAN_DEBUG

            auto new_sample = std::chrono::system_clock::now();

            std::ofstream ofs;
            ofs.open(fs::path{"/tmp/rikfan"} / zone->name);
            if (ofs.is_open())
            {
                ofs << "setpoint: " << zone->setpt;
                ofs << "\ninput:    " << input;
                ofs << "\noutput:   " << output;
                ofs << "\ninput time: " << get_input_time.count();
                std::chrono::duration<double> diff = new_sample - zone->sample_time;
                zone->sample_time = new_sample;
                ofs << "\nsample_time: " << diff.count();
                ofs << "\nmanualmode: " << zone->manualmode;
                ofs << "\nsensors: " << zone->sensors.size();
                ofs << "\nsensors[0].repr: " << zone->sensors[0]->repr();
                ofs << "\npwms: " << zone->pwms.size();
                ofs << "\npwms[0].repr: " << zone->pwms[0]->repr();
                ofs << "\nmanualmode: " << zone->manualmode;
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

void Zone::setPWM(unsigned int mode)
{
    for (auto &pwm : pwms)
    {
        pwm->set_value(double(mode));
    }
}

void Zone::setPwmInv(const std::vector<std::string> &inverted)
{
    if(inverted.size() == 0)
        return;

    for(const auto &pwm : pwms)
    {
        auto inv = std::find(inverted.cbegin(), inverted.cend(), pwm->get_path());
        if(inv != inverted.end())
        {
            pwm->set_inverted(true);
        }
    }
}




/**
 * ZoneManager
 */

ZoneManager::ZoneManager(fs::path conf_fname, boost::asio::io_service& io_)
{
    if (!fs::exists(conf_fname))
    {
        conf_fname = "/tmp/rikfan/conf.json";
        if (!fs::exists(conf_fname))
        {
            std::cerr << "Need config file in '/etc/rikfan/conf.json' or '/tmp/rikfan/conf.json'";
            return;
        }
    }

    conn = std::make_shared<sdbusplus::asio::connection>(io, sdbusplus::bus::new_system().release());

#ifdef RIKFAN_DEBUG
    {
        fs::path debug_path("/tmp/rikfan");
        if (!fs::exists(debug_path))
        {
            fs::create_directory(debug_path);
        }
    }
#endif // RIKFAN_DEBUG

    std::ifstream conf_stream {conf_fname};
    json conf_json;
    try
    {
        conf_stream >> conf_json;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        // syslog(LOG_ERR, "exception: %s", e.what());
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

            zones.emplace_back(std::make_unique<Zone>(zone_name, zone_type, pid_conf, sens_vect, pwm_vect, setpoint, pid_conf.ts * 1000, conn));
        }
    }
}

int ZoneManager::rawPWM(unsigned int perc)
{
    int readVal;
    try
    {
        readVal = perc * 255.0 / 100.0;
    }
    catch (const std::exception &e)
    {
        readVal = nomPWMraw;
    }
    return readVal;
}


void ZoneManager::setFanMode(unsigned int mode)
{
    if (mode == 0)
    {
        for (const auto &z : zones)
            z->command("auto");
    }
    else
    {
        int readVal;
        if(mode >= (sizeof(fanmode_values) / sizeof(fanmode_values[0])))
        {
            readVal = rawPWM(mode);
        }
        else
        {
            readVal = fanmode_values[mode];
        }

        for (const auto &z : zones)
        {
            z->command("manual");
            // Вручную установить значения PWM
            z->setPWM(readVal);
        }
        // setPWM(mode);
    }
}


void ZoneManager::start()
{
    // Запускаем циклы управления вентиляторами по зонам
    for (auto &z : zones)
        z->start();
}

void ZoneManager::setPwmInv(const std::vector<std::string> &inverted)
{
    for(auto &z : zones)
    {
        z->setPwmInv(inverted);
    }
}
