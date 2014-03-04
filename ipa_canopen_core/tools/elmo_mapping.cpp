/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: ipa_canopen
 * \note
 *   ROS stack name: ipa_canopen
 * \note
 *   ROS package name: ipa_canopen_core
 *
 * \author
 *   Author: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 * \author
 *   Supervised by: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: September 2013
 *
 * \brief
 *   This executable maps the Elmo 6063 e 6069h
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <utility>
#include <iostream>
#include <iomanip>
#include "canopen.h"
#include <sstream>

int main(int argc, char *argv[])
{

    if (argc != 4) {
        std::cout << "Arguments:" << std::endl
        << "(1) device file" << std::endl
        << "(2) CAN deviceID" << std::endl
        << "(3) Baud Rate" << std::endl
        << "Example: ./elmo_mapping /dev/pcan32 12 500K" << std::endl;
        return -1;
    }



    canopen::NMTmsg.ID = 0;
    canopen::NMTmsg.MSGTYPE = 0x00;
    canopen::NMTmsg.LEN = 2;

    canopen::syncMsg.ID = 0x80;
    canopen::syncMsg.MSGTYPE = 0x00;

    canopen::syncMsg.LEN = 0x00;

    std::string deviceFile = std::string(argv[1]);
    canopen::baudRate = std::string(argv[3]);

    if (!canopen::openConnection(deviceFile,canopen::baudRate)){
        std::cout << "Cannot open CAN device; aborting." << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Connection to CAN bus established" << std::endl;
    }

    uint16_t CANid = std::stoi(std::string(argv[2]));

    canopen::devices[ CANid ] = canopen::Device(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen::sendNMT(CANid, canopen::NMT_RESET_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));;
    canopen::sendNMT(CANid, canopen::NMT_START_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::setObjects();

    TPCANMsg mes;

    std::cout << "Initialized the PDO mapping" << std::endl;

    for(int pdo_object=1;pdo_object<=4;pdo_object++)
    {
        canopen::disableTPDO(pdo_object);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        canopen::clearTPDOMapping(pdo_object);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        canopen::disableRPDO(pdo_object);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        canopen::clearRPDOMapping(pdo_object);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if(canopen::use_limit_switch)
        canopen::makeTPDOMapping(1,"604100", 0x10, "60FD00", 0x20);
    else
        canopen::makeTPDOMapping(1,"604100", 0x10, "60FD00", 0x0);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    canopen::makeTPDOMapping(4, "606400", 0x20, "606C00", 0x20);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    canopen::makeRPDOMapping(1, "604000", 0x10, "60C101", 0x20);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    canopen::makeRPDOMapping(2, "60C101", 0x20, "604000", 0x10);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for(int pdo_object=1;pdo_object<=4;pdo_object++)
    {
        canopen::enableTPDO(pdo_object);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        canopen::enableRPDO(pdo_object);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

}
