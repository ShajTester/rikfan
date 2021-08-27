#pragma once

#include "pid.hpp"

#include <limits>
// #include <phosphor-logging/log.hpp>
// #include <sdbusplus/bus.hpp>
#include <string>

/* This program assumes sensors use the Sensor.Value interface
 * and for sensor->write() I only implemented sysfs as a type,
 * but -- how would it know whether to use Control.FanSpeed or Control.FanPwm?
 *
 * One could get the interface list for the object and search for Control.*
 * but, it needs to know the maximum, minimum.  The only sensors it wants to
 * write in this code base are Fans...
 */
enum class IOInterfaceType
{
    NONE, // There is no interface.
    EXTERNAL,
    DBUSPASSIVE,
    DBUSACTIVE, // This means for write that it needs to look up the interface.
    SYSFS,
    UNKNOWN
};

/* WriteInterfaceType is different because Dbusactive/passive. how to know... */
IOInterfaceType getWriteInterfaceType(const std::string& path);

IOInterfaceType getReadInterfaceType(const std::string& path);

void restartControlLoops(void);

/*
 * Given a configuration structure, fill out the information we use within the
 * PID loop.
 */
void initializePIDStruct(ec::pid_info_t* info, const ec::pidinfo& initial);

void dumpPIDStruct(std::ostream &os, ec::pid_info_t* info);

struct SensorProperties
{
    int64_t scale;
    double value;
    double min;
    double max;
    std::string unit;
};

struct SensorThresholds
{
    double lowerThreshold = std::numeric_limits<double>::quiet_NaN();
    double upperThreshold = std::numeric_limits<double>::quiet_NaN();
};


/*
 * Given a path that optionally has a glob portion, fill it out.
 */
std::string FixupPath(std::string original);
