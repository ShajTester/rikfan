/*
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "rikfan.hpp"
#include <phosphor-logging/log.hpp>
#include "log-conf.hpp"


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
    sdbusplus::asio::object_server server(conn);

    RikfanMgr rikfanMgr(io, server, conn);

    io.run();
}
