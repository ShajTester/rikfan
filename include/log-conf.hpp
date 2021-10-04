#pragma once

static constexpr const char* redfish_message_id = "OpenBMC.0.1.RikfanSensor";

void logger_send(int loglevel, const std::string &sensor_type, const std::string &sensor_address);
