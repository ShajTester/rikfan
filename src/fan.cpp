


#include <iostream>
#include <fstream>
#include <filesystem>
#include <exception>
#include <string>
#include <vector>

#include <cmath>
#include <cstdio>
#include <cstring>

#include <unistd.h>
#include <dirent.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/stat.h>


// #include <nlohmann/json.hpp>

namespace fs = std::filesystem;

// using json = nlohmann::json;
using namespace std::chrono_literals;


#define CONFIG_FILE_NAME "/usr/local/fan.conf"
#define minPWMraw   (70)
#define nomPWMraw   (140)
#define maxPWMraw   (255)
#define PWMFileFmt  "/sys/bus/platform/devices/1e786000.pwm-tacho-controller/hwmon/hwmon0/pwm%d"
#define FanFileFmt  "/sys/bus/platform/devices/1e786000.pwm-tacho-controller/hwmon/hwmon0/fan%d_input"


struct FANDescr
{
    std::string name;
    std::string webname;
    int tach;
    int pwm;
};

const std::vector<FANDescr> fanDescr
{
    {"Fan 1", "fan1", 1, 1},
    {"Fan 2", "fan2", 2, 2},
    {"Fan 3", "fan3", 3, 3},
    {"Fan 4", "fan4", 4, 4},
    {"Fan 5", "fan5", 8, 5},
    {"Fan 6", "fan6", 7, 6},
    {"CPU 0", "fan7", 6, 7},
    {"CPU 1", "fan8", 5, 8}
};


const int fanmode_values[] = {minPWMraw, minPWMraw, nomPWMraw, maxPWMraw};


int rawPWM(std::string perc)
{
    int readVal;
    try
    {
        readVal = std::lround(std::stoi(perc) * 255.0 / 100.0);
    }
    catch (const std::exception &e)
    {
        readVal = nomPWMraw;
    }
    return readVal;
}

int rawPWM(int perc)
{
    if(perc < 0)
        return 0;
    if(perc > 100)
        return 255;
    return std::lround(perc * 255.0 / 100.0);
}

std::string percPWM(int raw)
{
    if (raw < 0)
        return std::to_string(raw);
    return std::to_string(std::lround(raw * 100.0 / 255.0));
}


std::string percFAN(int raw)
{
    return std::to_string(raw);
}

#if 0
int fillState(json &jdata)
{
    std::ifstream ifd;
    int readVal;
    json pFT;
    char fname[256];

    jdata["fantach"] = json::array();
    for (const auto &it : fanDescr)
    {
        pFT = json::array();
        pFT += it.name;

        readVal = 0;
        std::sprintf(fname, FanFileFmt, it.tach);
        ifd.open(fname);
        if (ifd.is_open())
        {
            ifd >> readVal;
            if (!ifd.good())
                readVal = -1;
            ifd.close();
        }
        else
        {
            readVal = -2;
        }
        pFT += percFAN(readVal);

        readVal = 0;
        std::sprintf(fname, PWMFileFmt, it.pwm);
        ifd.open(fname);
        if (ifd.is_open())
        {
            ifd >> readVal;
            if (!ifd.good())
                readVal = -1;
            ifd.close();
        }
        pFT += percPWM(readVal);

        jdata["fantach"].emplace_back(pFT);
    }
    return 0;
}



/**
 * Для именованых каналов
 */
void setFansForList()
{
    int num;
    json newPwm;
    // Изменение задания в ручном режиме
    for (const auto &it : fanDescr)
    {
        // Чтение текущих значений PWM
        std::sprintf(cstr, PWMFileFmt, it.pwm);
        fd.open(cstr, std::ios::in);
        if (fd.is_open())
        {
            fd >> readVal;
            if (!fd.good())
                readVal = nomPWMraw;
            fd.close();
        }
        else
        {
            readVal = nomPWMraw;
        }

        if (jin.count(it.webname) > 0)
        {
            readVal = rawPWM(jin[it.webname].get<std::string>());
        }

        if (readVal < minPWMraw)
            readVal = minPWMraw;

        std::sprintf(cstr, PWMFileFmt, it.pwm);
        fd.open(cstr, std::ios::out);

        // Установить значения ШИМ
        if (fd.is_open())
        {
            fd << readVal;
            if (!fd.good())
            {
                syslog(LOG_ERR, "Error %d write to file <%s>", errno, cstr);
            }
            fd.close();
        }
        else
        {
            syslog(LOG_ERR, "Error %d open file <%s>", errno, cstr);
        }

        newPwm["pwm" + std::to_string(it.pwm)] = readVal;
    } // for

    // Сохранить в файле конфигурации новые значения.
    // Они будут установлены при перезагрузке.
    std::ofstream confout {CONFIG_FILE_NAME};
    confout << newPwm;
    confout.close();
}
#endif

/**
 * Для конфигурации одной для всех
 *
 * fanmode от 0% до 100% преобразуется в PWM от 0 до 255
 */
void setPWM(unsigned int fanmode)
{
    char cstr[256];
    std::fstream fd;
    int readVal;

    if(fanmode == 0)
    {
        readVal = rawPWM(0);
    }
    else if(fanmode >= (sizeof(fanmode_values) / sizeof(fanmode_values[0])))
    {
        readVal = rawPWM(fanmode);
    }
    else
    {
        readVal = fanmode_values[fanmode];
    }

    // Изменение задания в ручном режиме
    for (const auto &it : fanDescr)
    {
        std::sprintf(cstr, PWMFileFmt, it.pwm);
        fd.open(cstr, std::ios::out);

        // Установить значения ШИМ
        if (fd.is_open())
        {
            fd << readVal;
            if (!fd.good())
            {
                syslog(LOG_ERR, "Error %d write to file <%s>", errno, cstr);
            }
            fd.close();
        }
        else
        {
            syslog(LOG_ERR, "Error %d open file <%s>", errno, cstr);
        }
    }
}

