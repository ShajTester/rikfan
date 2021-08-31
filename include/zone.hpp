
#pragma once

#define RIKFAN_DEBUG

#include <boost/asio/io_service.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include "pid.hpp"
#include "util.hpp"

#include <vector>
#include <filesystem>
namespace fs = std::filesystem;


class SensorBase
{
public:
    virtual double get_value() = 0;
    virtual void set_value(double in) = 0;
    virtual std::string repr() = 0;
};


class SensorFS: public SensorBase
{
public:
    SensorFS(const std::string &p, double err_val);
    double get_value() override;
    void set_value(double in) override;
    std::string repr() override;
private:
    std::string real_path;
    int state;
    double error_value;
};


class SensorDBus: public SensorBase
{
public:
    SensorDBus(const std::string &p, double err_val, std::shared_ptr<sdbusplus::asio::connection>& b);
    double get_value() override ;
    void set_value(double in) override {throw "Not implemented yet.";}
    std::string repr() override;
private:
    std::string real_path;
    int state;
    double error_value;
    std::shared_ptr<sdbusplus::asio::connection> passive_bus;
};





class Zone
{
private:
	static const constexpr long long loop_min_delay = 300;
	static const constexpr double stop_output_const = 130.0;
	static const constexpr double margin_error_read_temp = 100.0;

public:

	// Zone() = delete;
	Zone(const Zone &) = delete;
	Zone(Zone &&) = delete;
	void operator=(const Zone&) = delete;
	void operator=(Zone&&) = delete;

	Zone(std::string n,
	     std::string t,
	     ec::pidinfo &pidinfo_initial,
	     std::vector<std::string> &s,
	     std::vector<std::string> &f,
	     double sp,
	     long long ms,
	     std::shared_ptr<sdbusplus::asio::connection>& conn_
	    );

	~Zone();
	void start();
	void stop();
	void command(const char *cmd);


private:
	bool manualmode;
	bool stop_flag;
	long long millisec;
	std::unique_ptr<std::thread> pmainthread;

	std::string name;
	std::string type;
	ec::pid_info_t pid_info;
	double setpt;
	std::vector<std::unique_ptr<SensorBase>> sensors;
	std::vector<std::unique_ptr<SensorBase>> pwms;
	double error_read_temp;
	std::shared_ptr<sdbusplus::asio::connection> conn;

#ifdef RIKFAN_DEBUG
	decltype(std::chrono::system_clock::now()) sample_time;
#endif

	double processInputs();
	double processPID(double in);
	void processOutputs(double in);
	static void zone_control_loop(Zone *zone);
};


class ZoneManager
{
	/* async io context for operation */
	boost::asio::io_context io;

	std::shared_ptr<sdbusplus::asio::connection> conn;
	std::vector<std::unique_ptr<Zone>> zones;

public:
	// ZoneManager() = delete;
	ZoneManager(const ZoneManager &) = delete;
	ZoneManager(ZoneManager &&) = delete;
	void operator=(const ZoneManager&) = delete;
	void operator=(ZoneManager&&) = delete;

	ZoneManager(fs::path conf_fname, boost::asio::io_service& io_);

	void setFanMode(unsigned int mode);
	void start();

};