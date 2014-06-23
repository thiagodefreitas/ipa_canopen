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
 *   Author: Thiago de Freitas, Tobias Sing, Eduard Herkel
 * \author
 *   Supervised by: Thiago de Freitas email:tdf@ipa.fhg.de
 *
 * \date Date of creation: December 2012
 *
 * \brief
 *   Implementation of canopen driver.
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
//! Including header files

#include <ipa_canopen_core/canopen.h>
#include <sstream>
#include <cstring>
#include <unordered_map>
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
#include<algorithm>

=======
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

namespace canopen
{

/***************************************************************/
//!			define global variables and functions
/***************************************************************/
bool sdo_protect=false;
BYTE protect_msg[8];

//!Defining SyncInterval, Baudrate, devices and device groups
std::chrono::milliseconds syncInterval;
std::string baudRate;
std::map<uint8_t, Device> devices;
std::map<std::string, DeviceGroup> deviceGroups;

//! Create a handle to the device node
HANDLE h;
std::vector<std::thread> managerThreads;
std::vector<std::string> openDeviceFiles;
bool atFirstInit=true;
int initTrials=0;
std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingDataHandlers { { STATUSWORD, sdo_incoming }, { DRIVERTEMPERATURE, sdo_incoming }, { MODES_OF_OPERATION_DISPLAY, sdo_incoming } };
std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingErrorHandlers { { ERRORWORD, errorword_incoming }, { MANUFACTURER, errorword_incoming } };
std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingManufacturerDetails { {MANUFACTURERHWVERSION, manufacturer_incoming}, {MANUFACTURERDEVICENAME, manufacturer_incoming}, {MANUFACTURERSOFTWAREVERSION, manufacturer_incoming} };
std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;
std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingEMCYHandlers;
bool recover_active;
bool halt_active;

bool halt_positive;
bool halt_negative;

bool use_limit_switch=false;

std::string operation_mode_param;

std::chrono::time_point<std::chrono::high_resolution_clock> start, end;

std::chrono::duration<double> elapsed_seconds;


/***************************************************************/
//!		define init and recover sequence
/***************************************************************/


<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp

=======
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
//! Function to Open a Connection to a CAN port
/*! The function takes 2 parameters
 * std::string devName is the name of the device
 * std::string baudrate is the baudrate of the device*/
bool openConnection(std::string devName, std::string baudrate)
{
	//!Create a path to a CAN port
	/*! input: the path to the device node (e.g. /dev/pcan0) and an int flag (O_RDWR)
       returns NULL when open fails */
    h = LINUX_CAN_Open(devName.c_str(), O_RDWR);
    if (!h)
        return false;
	
	/*! initializes the CAN hardware with the baudrates[baudrate] constant "CAN_BAUD_...".
	 * nCANMsgType must be filled with "CAN_INIT_TYPE_.."*/	 
    errno = CAN_Init(h, baudrates[baudrate], CAN_INIT_TYPE_ST);

    return true;
}

//!Function for Pre-Initialising the device
/*! The function takes no arguments and returns no arguments*/
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
void pre_init(std::string chainName)
{
	//! For all devices in the chain
    for (auto id : canopen::deviceGroups[chainName].getCANids())
=======
void pre_init()
{
	//! For all devices 
    for (auto device : (canopen::devices))
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
    {
        /*********************************************/
		//! Get all error information from the error registers
		/*********************************************/
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
        getErrors(id);
=======
		getErrors(device.second.getCANid());
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

        /***************************************************************/
        //!		Manufacturer specific errors register
        /***************************************************************/
        readManErrReg(id);

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
        /**************************
       //! Get Hardware and Software Information
      /************************/
        canopen::uploadSDO(id, MANUFACTURERDEVICENAME);
=======
        /**************************/
        //! Get Hardware and Software Information
        /*************************/
        canopen::uploadSDO(device.second.getCANid(), MANUFACTURERDEVICENAME);
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
    }
}
//////////////////////////////////////
/////////////

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp

//! Function to initialize the device
/*! The function takes 3 parameters 
 * deviceFile is the name of the device
 * chainName is the name of the chain
 * int8_t mode_of_operation is the mode of operation of the device */
bool init(std::string deviceFile, std::string chainName, const int8_t mode_of_operation)
=======
//! Function to initialize the device
/*! The function takes 2 parameters 
 * deviceFile is the name of the device
 * syncInterval is the syncinterval of the device */
bool init(std::string deviceFile, std::chrono::milliseconds syncInterval)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    initTrials++;
	//! If initialization does not happen for 4 times, then reset the devices
    if(initTrials == 4)
    {
        std::cout << "There are still problems with the devices. Trying a complete reset " << std::endl;
        canopen::sendNMT(0x00, canopen::NMT_RESET_NODE);

        initTrials=0;
    }

    if(canopen::atFirstInit)
    {
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
		
        canopen::atFirstInit = false;

        bool connection_success;
		//! Check if connection to the device is available
        bool connection_is_available = std::find(canopen::openDeviceFiles.begin(), canopen::openDeviceFiles.end(), deviceFile) != canopen::openDeviceFiles.end();
        
		
        if(!connection_is_available)
=======
		//!If atFirstInit is already true, set it to false
		atFirstInit = false;
		
		/*! CAN_Close(h)  closes the path to the CAN hardware.
		 * The last close on the hardware put the chip into passive state*/
        CAN_Close(h);

        canopen::initDeviceManagerThread(canopen::deviceManager);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        bool connection_success;
        //! Open the connection for the CAN device
        connection_success = canopen::openConnection(deviceFile, canopen::baudRate);
        
        //! If connection is not established, display an error message
        if (!connection_success)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
        {
			//! If connection is not available, then close the CAN port and try to open the connection again
            CAN_Close(canopen::h);
            connection_success = canopen::openConnection(deviceFile, canopen::baudRate);
			
			//! If its not possible to open the CAN connection, then abort after displaying a message
            if (!connection_success)
            {
                std::cout << "Cannot open CAN device "<< deviceFile << "; aborting." << std::endl;
                exit(EXIT_FAILURE);
            }
            canopen::initListenerThread(canopen::defaultListener);
            canopen::openDeviceFiles.push_back(deviceFile);
        }
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp

        std::cout << "Resetting communication with the devices " << std::endl;
        canopen::sendNMT(0x00, canopen::NMT_RESET_COMMUNICATION);

    }


    if(canopen::deviceGroups[chainName].getFirstInit())
    {

        std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;
        time_start = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds;
        
        canopen::initDeviceManagerThread(chainName,canopen::deviceManager);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
=======
        
        //! Try to initialize the device if connection is established
        else
        {

                canopen::initListenerThread(canopen::defaultListener);
				
				//! Preinitialze the device
                canopen::pre_init();
				
                while(sdo_protect)
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    
                //!Reset the network management node
                std::cout << "Resetting devices " << std::endl;
                canopen::sendNMT(0x00, canopen::NMT_RESET_NODE);
                
                //!For all the devices , make the node available by initializing the node
                for(auto device : devices)
                {
                    bool nmt_init = devices[device.second.getCANid()].getNMTInit();
                    std::cout << "Waiting for Node: " << (uint16_t)device.second.getCANid() << " to become available" << std::endl;
                    
                    while(!nmt_init)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        nmt_init = devices[device.second.getCANid()].getNMTInit();
                    }
                    std::cout << "Node: " << (uint16_t)device.second.getCANid() << " is now available" << std::endl;
                }
                
                canopen::sendNMT(0x00, canopen::NMT_START_REMOTE_NODE);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

		//! Initialize the PDOmapping for all devices
        for (auto device : devices)
        {
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

        std::cout << "Initializing " << chainName << std::endl;

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
        canopen::deviceGroups[chainName].setFirstInit(false);

        canopen::pre_init(chainName);

        while(sdo_protect)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
         elapsed_seconds = time_end - time_start;

            if(elapsed_seconds.count() > 5.0)
                {
                    std::cout << "not ready for operation. Probably due to communication problems with the Master." << std::endl;
                    return false;
                }
                time_end = std::chrono::high_resolution_clock::now();
        }

        time_start = std::chrono::high_resolution_clock::now();

        
        
        for(auto id : canopen::deviceGroups[chainName].getCANids())
        {
=======
            for(int pdo_object=0; pdo_object<=3; pdo_object++)
            {
                canopen::disableTPDO(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                canopen::clearTPDOMapping(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                canopen::disableRPDO(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                canopen::clearRPDOMapping(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

			//! use_limit_switch is always set to false. So 'else' statement is always executed
            if(canopen::use_limit_switch)
            {
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

            bool nmt_init = devices[id].getNMTInit();
            std::cout << "Waiting for Node: " << (uint16_t)id << " to become available" << std::endl;

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp

            while(!nmt_init)
            {
                elapsed_seconds = time_end - time_start;

                if(elapsed_seconds.count() > 25.0)
                {
                    std::cout << "Node: " << (uint16_t)id << " is not ready for operation. Please check for eventual problems." << std::endl;
                    return false;
                }
=======
                canopen::makeTPDOMapping(0,tpdo1_registers, tpdo1_sizes, u_int8_t(0xFF));
            }
            
            //! Initialize the tpdo1_registers and tpdo1_sizes arrays according to required PDO mapping data required
            else
            {
                std::vector<std::string> tpdo1_registers {"604100", "606100"};
                std::vector<int> tpdo1_sizes {0x10,0x08};
				
                canopen::makeTPDOMapping(0,tpdo1_registers, tpdo1_sizes, u_int8_t(0xFF));
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                nmt_init = devices[id].getNMTInit();
                time_end = std::chrono::high_resolution_clock::now();
            }
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp

            std::cout << "Node: " << (uint16_t)id << " is now available" << std::endl;

            canopen::sendNMT((u_int8_t)id, canopen::NMT_START_REMOTE_NODE);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
=======
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
			
			//! Initialize the tpdo4 registers and their sizes and make the required mapping
            std::vector<std::string> tpdo4_registers {"606400", "606C00"};
            std::vector<int> tpdo4_sizes {0x20,0x20};

            canopen::makeTPDOMapping(3, tpdo4_registers, tpdo4_sizes, u_int8_t(0x01));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

			//! Initialize the rpdo1 registers and their sizes 
            std::vector<std::string> rpdo1_registers {"604000"};
            std::vector<int> rpdo1_sizes {0x10};
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

	//! Initialize the PDOmapping for all devices
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
        std::cout << "Initialized the PDO mapping for Node:" << (uint16_t)id << std::endl;

        for(int pdo_object=0; pdo_object<=3; pdo_object++)
        {
            canopen::disableTPDO(chainName,pdo_object);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            canopen::clearTPDOMapping(chainName, pdo_object);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            canopen::disableRPDO(chainName,pdo_object);
=======
			//! Make the required mapping for the RPDOs rpdo1 and rpdo2
            canopen::makeRPDOMapping(0, rpdo1_registers, rpdo1_sizes, u_int8_t(0xFF));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            canopen::makeRPDOMapping(1, rpdo2_registers, rpdo2_sizes, u_int8_t(0x01));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
			
			//! For all the 4 pdo_objects, enable the RPDOs and TPDOs
            for(int pdo_object=0; pdo_object<=3; pdo_object++)
            {
                canopen::enableTPDO(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                canopen::enableRPDO(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            canopen::clearRPDOMapping(chainName, pdo_object);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
		//! use_limit_switch is always set to false. So 'else' statement is always executed
        if(canopen::use_limit_switch)
        {

            std::vector<std::string> tpdo1_registers {"604100", "60FD00"};
            std::vector<int> tpdo1_sizes {0x10,0x20};

            canopen::makeTPDOMapping(chainName,0,tpdo1_registers, tpdo1_sizes, u_int8_t(0xFF));
        }
         //! Initialize the tpdo1_registers and tpdo1_sizes arrays according to required PDO mapping data required
        else
        {
            std::vector<std::string> tpdo1_registers {"604100", "606100"};
            std::vector<int> tpdo1_sizes {0x10,0x08};

            canopen::makeTPDOMapping(chainName,0,tpdo1_registers, tpdo1_sizes, u_int8_t(0xFF));

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

		//! Initialize the tpdo4 registers and their sizes and make the required mapping
        std::vector<std::string> tpdo4_registers {"606400", "606C00"};
        std::vector<int> tpdo4_sizes {0x20,0x20};

        canopen::makeTPDOMapping(chainName,3, tpdo4_registers, tpdo4_sizes, u_int8_t(0x01));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
	
		//! Initialize the rpdo1 registers and their sizes 
        std::vector<std::string> rpdo1_registers {"604000"};
        std::vector<int> rpdo1_sizes {0x10};

		
        std::vector<std::string> rpdo2_registers {"60C101"};
        std::vector<int> rpdo2_sizes {0x20};
        
		//! Make the required mapping for the RPDOs rpdo1 and rpdo2
        canopen::makeRPDOMapping(chainName,0, rpdo1_registers, rpdo1_sizes, u_int8_t(0xFF));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        canopen::makeRPDOMapping(chainName,1, rpdo2_registers, rpdo2_sizes, u_int8_t(0x01));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

		//! For all the 4 pdo_objects, enable the RPDOs and TPDOs
        for(int pdo_object=0; pdo_object<=3; pdo_object++)
        {
            canopen::enableTPDO(chainName, pdo_object);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            canopen::enableRPDO(chainName, pdo_object);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    }

   }
    recover_active = false;

    canopen::setObjects(chainName);

=======
    for (auto device : devices)
    {
		//!Send the Sync signal
        canopen::sendSync();
        
        //!Check whether the Motor state is already operational. If so, display the appropriate message. 
        if(device.second.getMotorState() == MS_OPERATION_ENABLED)
        {
            std::cout << "Node" << (uint16_t)device.second.getCANid() << "is already operational" << std::endl;
        }
        
       
        else
        {
			 //! If motor is not operational, set the mode of operation to Interpolated_Position_Mode
            canopen::sendSDO(device.second.getCANid(), canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			
			//! Set the Motor state Constant to MS_SWITCHED_ON_DISABLED
            canopen::setMotorState((uint16_t)device.second.getCANid(), canopen::MS_SWITCHED_ON_DISABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

			//! Set the Motor state Constant to MS_READY_TO_SWITCH_ON
            canopen::setMotorState((uint16_t)device.second.getCANid(), canopen::MS_READY_TO_SWITCH_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

			//! Set the Motor state Constant to MS_SWITCHED_ON
            canopen::setMotorState((uint16_t)device.second.getCANid(), canopen::MS_SWITCHED_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

			//! Set the Motor state Constant to MS_OPERATION_ENABLED
            canopen::setMotorState((uint16_t)device.second.getCANid(), canopen::MS_OPERATION_ENABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

			//! Store the syncinterval in IP_TIME_UNITS
            sendSDO((uint16_t)device.second.getCANid(), canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            //! Store the IP_TIME_INDEX_MILLISECONDS in IP_TIME_INDEX
            sendSDO((uint16_t)device.second.getCANid(), canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            //! Store SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT in SYNC_TIMEOUT_FACTOR
            sendSDO((uint16_t)device.second.getCANid(), canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));


            canopen::controlPDO(device.second.getCANid(), canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            //! Set the desired Position as the actual position and desired velocity as 0
            canopen::devices[device.second.getCANid()].setDesiredPos((double)device.second.getActualPos());
            canopen::devices[device.second.getCANid()].setDesiredVel(0);
            
            //!Necessary otherwise sometimes Schunk devices complain for Position Track Error
            sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());
			
            canopen::uploadSDO(device.second.getCANid(), canopen::STATUSWORD);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            canopen::uploadSDO(device.second.getCANid(), DRIVERTEMPERATURE);
            canopen::uploadSDO(device.second.getCANid(), MODES_OF_OPERATION_DISPLAY);
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        bool set_operation_mode = canopen::setOperationMode(id, mode_of_operation);
        
        if(!set_operation_mode)
            return false;
        canopen::setMotorState((uint16_t)id, canopen::MS_OPERATION_ENABLED);

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
        //!Necessary otherwise sometimes Schunk devices complain for Position Track Error
        canopen::devices[id].setDesiredPos((double)devices[id].getActualPos());
        canopen::devices[id].setDesiredVel(0);

        sendPos((uint16_t)id, (double)devices[id].getDesiredPos());

        canopen::controlPDO(id, canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        getErrors(id);
        readManErrReg(id);
=======
    }
	
	//!Check for all nodes that the driver side init is successful
    for (auto device : devices)
    {
        getErrors(device.second.getCANid());
        readManErrReg(device.second.getCANid());
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if(devices[id].getIPMode())
        {
            std::cout << "Concluded driver side init succesfully for Node" << (uint16_t)id << std::endl;
            canopen::devices[id].setInitialized(true);
        }
        else
        {
            std::cout << "Problems occured during driver side init for Node" << (uint16_t)id  << std::endl;
            canopen::devices[id].setInitialized(false);
            return false;
        }

    }
	
    return true;
}

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
bool init(std::string deviceFile, std::string chainName, std::chrono::milliseconds syncInterval)
{
    bool initialized = init(deviceFile, chainName, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        sendSDO((uint16_t)id, canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        sendSDO((uint16_t)id, canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        sendSDO((uint16_t)id, canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return initialized;
}

//! Function to recover the device
/*! The function takes 2 parameters 
 * deviceFile is the name of the device
 * chainName is the name of the chain
 * syncInterval is the syncinterval of the device */
bool recover(std::string deviceFile, std::string chainName, std::chrono::milliseconds syncInterval)
=======
//! Function to recover the device
/*! The function takes 2 parameters 
 * deviceFile is the name of the device
 * syncInterval is the syncinterval of the device */
bool recover(std::string deviceFile, std::chrono::milliseconds syncInterval)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
	//!Set recover_active to true initially
    recover_active = true;

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
        if(devices[id].getIPMode())
=======
        if(device.second.getIPMode())
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
        {
            std::cout << "Node" << id << "is already operational" << std::endl;
        }
        else
        {
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp

            canopen::controlPDO(id,canopen::CONTROLWORD_HALT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(id,canopen::CONTROLWORD_DISABLE_INTERPOLATED, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(id,canopen::CONTROL_WORD_DISABLE_VOLTAGE, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(id,canopen::CONTROLWORD_QUICKSTOP, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//!Set the mode of operation to Interpolated_Position_Mode
            canopen::sendSDO(id, canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Set the Motor state Constant to MS_SWITCHED_ON_DISABLED
            canopen::setMotorState(id, canopen::MS_SWITCHED_ON_DISABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Set the Motor state Constant to MS_READY_TO_SWITCH_ON
            canopen::setMotorState(id, canopen::MS_READY_TO_SWITCH_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Set the Motor state Constant to MS_SWITCHED_ON
            canopen::setMotorState(id, canopen::MS_SWITCHED_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Set the Motor state Constant to MS_OPERATION_ENABLED
            canopen::setMotorState(id, canopen::MS_OPERATION_ENABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Store the syncinterval in IP_TIME_UNITS
            sendSDO((uint16_t)id, canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Store the IP_TIME_INDEX_MILLISECONDS in IP_TIME_INDEX
            sendSDO((uint16_t)id, canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Store SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT in SYNC_TIMEOUT_FACTOR
            sendSDO((uint16_t)id, canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			
			//!Enable the movement of the device
            canopen::controlPDO(id, canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            canopen::uploadSDO(id, canopen::STATUSWORD);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            canopen::uploadSDO(id, DRIVERTEMPERATURE);
            canopen::uploadSDO(id, MODES_OF_OPERATION_DISPLAY);
=======
			
            canopen::controlPDO(device.second.getCANid(),canopen::CONTROLWORD_HALT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(device.second.getCANid(),canopen::CONTROLWORD_DISABLE_INTERPOLATED, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(device.second.getCANid(),canopen::CONTROL_WORD_DISABLE_VOLTAGE, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(device.second.getCANid(),canopen::CONTROLWORD_QUICKSTOP, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
			//!Set the mode of operation to Interpolated_Position_Mode
            canopen::sendSDO(device.second.getCANid(), canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			
			//! Set the Motor state Constant to MS_SWITCHED_ON_DISABLED
            canopen::setMotorState(device.second.getCANid(), canopen::MS_SWITCHED_ON_DISABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Set the Motor state Constant to MS_READY_TO_SWITCH_ON
            canopen::setMotorState(device.second.getCANid(), canopen::MS_READY_TO_SWITCH_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Set the Motor state Constant to MS_SWITCHED_ON
            canopen::setMotorState(device.second.getCANid(), canopen::MS_SWITCHED_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Set the Motor state Constant to MS_OPERATION_ENABLED
            canopen::setMotorState(device.second.getCANid(), canopen::MS_OPERATION_ENABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			//! Store the syncinterval in IP_TIME_UNITS
            sendSDO((uint16_t)device.second.getCANid(), canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
             //! Store the IP_TIME_INDEX_MILLISECONDS in IP_TIME_INDEX
            sendSDO((uint16_t)device.second.getCANid(), canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            //! Store SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT in SYNC_TIMEOUT_FACTOR
            sendSDO((uint16_t)device.second.getCANid(), canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			
			//!Enable the movement of the device
            canopen::controlPDO(device.second.getCANid(), canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
			
			
            canopen::uploadSDO(device.second.getCANid(), canopen::STATUSWORD);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            canopen::uploadSDO(device.second.getCANid(), DRIVERTEMPERATURE);
            canopen::uploadSDO(device.second.getCANid(), MODES_OF_OPERATION_DISPLAY);
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

            getErrors(id);
        }

		//! Set the desired Position as the actual position and desired velocity as 0
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
        devices[id].setDesiredPos((double)devices[id].getActualPos());
        devices[id].setDesiredVel(0);
=======
        devices[device.second.getCANid()].setDesiredPos((double)device.second.getActualPos());
        devices[device.second.getCANid()].setDesiredVel(0);
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

    }
    recover_active = false;

	//!Check for all nodes that the driver side recover is successful
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
    for (auto id : canopen::deviceGroups[chainName].getCANids())
=======
    for (auto device : devices)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
    {

        if(devices[id].getIPMode())
        {
            std::cout << "Concluded driver side recover succesfully" << std::endl;
        }
        else
        {
            std::cout << "Problems occured during driver side recover" << std::endl;
            return false;
        }
    }

    return true;

}
///////////////////////////////7
///

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
//! Function to halt the device in a chain
/*! The function takes 2 parameters 
 * deviceFile is the name of the device
 * chainName is the name of the chain
 * syncInterval is the syncinterval of the device */

void halt(std::string deviceFile, std::string chainName, std::chrono::milliseconds syncInterval)
=======
//! Function to halt the device
/*! The function takes 2 parameters 
 * deviceFile is the name of the device
 * syncInterval is the syncinterval of the device */

void halt(std::string deviceFile, std::chrono::milliseconds syncInterval)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
	/*! CAN_Close() closes the path to the CAN hardware
	 *  The last close on the hardware put the chip into passive state*/
    CAN_Close(h);

    NMTmsg.ID = 0;
    NMTmsg.MSGTYPE = 0x00;
    NMTmsg.LEN = 2;

    syncMsg.ID = 0x80;
    syncMsg.MSGTYPE = 0x00;

    syncMsg.LEN = 0x00;
	//!Try to open the connection for the device. If not successful, display an error message
    if (!canopen::openConnection(deviceFile, canopen::baudRate))
    {
        std::cout << "Cannot open CAN device; aborting." << std::endl;
        exit(EXIT_FAILURE);
    }
    //! If connection is established, display a success message
    else
    {
        //! std::cout << "Connection to CAN bus established (recover)" << std::endl;
    }

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        //! std::cout << "Module with CAN-id " << (uint16_t)id << " connected (recover)" << std::endl;
    }
	//!For all devices, halt the device by sending SDOs in a sequence
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {

		//!Send CONTROLWORD_HALT to CONTROLWORD
        canopen::sendSDO(id, canopen::CONTROLWORD, canopen:: CONTROLWORD_HALT);
		
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//! Send CONTROLWORD_DISABLE_INTERPOLATED to CONTROLWORD
        canopen::sendSDO(id, canopen::CONTROLWORD, canopen:: CONTROLWORD_DISABLE_INTERPOLATED);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//! Send CONTROL_WORD_DISABLE_VOLTAGE to CONTROLWORD
        canopen::sendSDO(id, canopen::CONTROLWORD, canopen:: CONTROL_WORD_DISABLE_VOLTAGE);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//! Send CONTROLWORD_QUICKSTOP to CONTROLWORD
        canopen::sendSDO(id, canopen::CONTROLWORD, canopen::CONTROLWORD_QUICKSTOP);
        canopen::uploadSDO(id, canopen::STATUSWORD);
=======
	
    for (auto device : devices)
    {
        //!std::cout << "Module with CAN-id " << (uint16_t)device.second.getCANid() << " connected (recover)" << std::endl;
    }
	
	//!For all devices, halt the device by sending SDOs in a sequence
    for (auto device : devices)
    {

		//!Send CONTROLWORD_HALT to CONTROLWORD
        canopen::sendSDO(device.second.getCANid(), canopen::CONTROLWORD, canopen:: CONTROLWORD_HALT);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//! Send CONTROLWORD_DISABLE_INTERPOLATED to CONTROLWORD
        canopen::sendSDO(device.second.getCANid(), canopen::CONTROLWORD, canopen:: CONTROLWORD_DISABLE_INTERPOLATED);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//! Send CONTROL_WORD_DISABLE_VOLTAGE to CONTROLWORD
        canopen::sendSDO(device.second.getCANid(), canopen::CONTROLWORD, canopen:: CONTROL_WORD_DISABLE_VOLTAGE);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//! Send CONTROLWORD_QUICKSTOP to CONTROLWORD
        canopen::sendSDO(device.second.getCANid(), canopen::CONTROLWORD, canopen::CONTROLWORD_QUICKSTOP);
        canopen::uploadSDO(device.second.getCANid(), canopen::STATUSWORD);
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}

/***************************************************************/
//!		define state machine functions
/***************************************************************/

void setNMTState(uint16_t CANid, std::string targetState)
{

}

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
bool setOperationMode(uint16_t CANid, const int8_t targetMode, double timeout)
{
    std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;

    time_start = std::chrono::high_resolution_clock::now();

    //! check if motor is in a legitimate state to change operation mode
    if (    devices[CANid].getMotorState() != MS_READY_TO_SWITCH_ON &&
            devices[CANid].getMotorState() != MS_SWITCHED_ON_DISABLED &&
            devices[CANid].getMotorState() != MS_SWITCHED_ON)
    {
        std::cout << "Found motor in state " << devices[CANid].getMotorState() << ", need to adjust state to SWITCHED_ON" << std::endl;
        setMotorState(CANid, canopen::MS_SWITCHED_ON);
    }

    //! change operation mode until correct mode is returned
    while (devices[CANid].getCurrentModeofOperation() != targetMode)
    {
        canopen::sendSDO(CANid, canopen::MODES_OF_OPERATION, (uint8_t)targetMode);
        canopen::uploadSDO(CANid, canopen::MODES_OF_OPERATION_DISPLAY);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        //! timeout check
        time_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = time_end-time_start;

        if(elapsed_seconds.count() > timeout)
        {
            std::cout << "setting operation mode failed" << std::endl;
            return false;
        }
    }

    return true;
}

=======
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
//!Function to set the Motor state constants
/*! The function takes 2 parameters
 * CANid is the CAN ID of the device 
 * targetState is the state of the motor to be achieved*/
void setMotorState(uint16_t CANid, std::string targetState)
{

    start = std::chrono::high_resolution_clock::now();

    while (devices[CANid].getMotorState() != targetState)
    {
        end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;

        if(elapsed_seconds.count() > 3)
            return;
        canopen::uploadSDO(CANid, canopen::STATUSWORD);
        if (devices[CANid].getMotorState() == MS_FAULT)
        {
            canopen::uploadSDO(CANid, canopen::STATUSWORD);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
			
            if(!devices[CANid].getFault())
            {
                canopen::controlPDO(CANid, canopen::CONTROLWORD_FAULT_RESET_0, 0x00);
            }
            else
            {
                //!std::this_thread::sleep_for(std::chrono::milliseconds(50));
                canopen::controlPDO(CANid, canopen::CONTROLWORD_FAULT_RESET_1, 0x00);
            }

        }

        if (devices[CANid].getMotorState() == MS_NOT_READY_TO_SWITCH_ON)
        {
            canopen::uploadSDO(CANid, canopen::STATUSWORD);
            canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN, 0x00);
        }

        if (devices[CANid].getMotorState() == MS_SWITCHED_ON_DISABLED)
        {
            //!canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_SHUTDOWN);
            canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN, 0x00);
        }

        if (devices[CANid].getMotorState() == MS_READY_TO_SWITCH_ON)
        {
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
            if (targetState == MS_SWITCHED_ON_DISABLED)
            {
                canopen::controlPDO(CANid, canopen::CONTROL_WORD_DISABLE_VOLTAGE, 0x00);
            }
            else
            {
                canopen::controlPDO(CANid, canopen::CONTROLWORD_SWITCH_ON, 0x00);
            }
=======
            //!canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_SWITCH_ON);
            canopen::controlPDO(CANid, canopen::CONTROLWORD_SWITCH_ON, 0x00);
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
        }

        if (devices[CANid].getMotorState() == MS_SWITCHED_ON)
        {
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
            if (targetState == MS_SWITCHED_ON_DISABLED)
            {
                canopen::controlPDO(CANid, canopen::CONTROL_WORD_DISABLE_VOLTAGE, 0x00);
            }
            else if (targetState == MS_READY_TO_SWITCH_ON)
            {
                canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN, 0x00);
            }
            else
            {
                canopen::controlPDO(CANid, canopen::CONTROLWORD_ENABLE_OPERATION, 0x00);
            }
        }

        if (devices[CANid].getMotorState() == MS_OPERATION_ENABLED)
        {
            if (targetState == MS_SWITCHED_ON_DISABLED)
            {
                canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROL_WORD_DISABLE_VOLTAGE);
            }
            else if (targetState == MS_READY_TO_SWITCH_ON)
            {
                canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_SHUTDOWN);
            }
            else
            {
                canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_DISABLE_OPERATION);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
=======
            //!canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_ENABLE_OPERATION);
            canopen::controlPDO(CANid, canopen::CONTROLWORD_ENABLE_OPERATION, 0x00);
        }

>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
    }
}

/***************************************************************/
//!			define NMT variables
/***************************************************************/

TPCANMsg NMTmsg;

/***************************************************************/
//!			define SYNC variables
/***************************************************************/

TPCANMsg syncMsg;

/***************************************************************/
//!		define SDO protocol functions
/***************************************************************/

//!Request for different blocks of data using non-expedited data transfer
void requestDataBlock1(uint8_t CANid)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x60;
    msg.DATA[1] = 0x00;
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h, &msg); //!Write to the CAN bus
    }
}

//!Request for different blocks of data using non-expedited data transfer with toggle set to 0
void requestDataBlock2(uint8_t CANid)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x70;
    msg.DATA[1] = 0x00;
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h, &msg);
}

//! To write the control word
void controlPDO(uint8_t CANid, u_int16_t control1, u_int16_t control2)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x200;
    msg.MSGTYPE = 0x00;
    msg.LEN = 2;
    msg.DATA[0] = control1;
    msg.DATA[1] = control2;
    CAN_Write(h, &msg);
}

//! Sets an SDO to 0
void uploadSDO(uint8_t CANid, SDOkey sdo)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x40;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h, &msg);
}

//!Expediate SDO upload of a unsigned value of 32 bits
void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x23;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = (value >> 16) & 0xFF;
    msg.DATA[7] = (value >> 24) & 0xFF;
    CAN_Write(h, &msg);
}

//!Expediate SDO upload of a signed value of 32 bits
void sendSDO(uint8_t CANid, SDOkey sdo, int32_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x23;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = (value >> 16) & 0xFF;
    msg.DATA[7] = (value >> 24) & 0xFF;
    CAN_Write(h, &msg);
}

//! Function sets unknown SDOs with the value in the 32 bit integer
void sendSDO_unknown(uint8_t CANid, SDOkey sdo, int32_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x22;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = (value >> 16) & 0xFF;
    msg.DATA[7] = (value >> 24) & 0xFF;
    CAN_Write(h, &msg);
}

//!Expediate SDO upload of an unsigned value of 8 bits
void sendSDO(uint8_t CANid, SDOkey sdo, uint8_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x2F;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h, &msg);


}

//!Expediate SDO upload of an unsigned value of 16 bits

void sendSDO(uint8_t CANid, SDOkey sdo, uint16_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x2B;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h, &msg);
}

/***************************************************************/
//!		define PDO protocol functions
/***************************************************************/

void initDeviceManagerThread(std::string chainName, std::function<void (std::string)> const& deviceManager)
{
    std::thread device_manager_thread(deviceManager, chainName);
    device_manager_thread.detach();
    //managerThreads.push_back(device_manager_thread);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void deviceManager(std::string chainName)
{
    std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;


    while (true)
    {
        time_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = time_end-time_start;

        auto tic = std::chrono::high_resolution_clock::now();
        if (!recover_active)
        {
            for (auto id : canopen::deviceGroups[chainName].getCANids())
            {
                if(elapsed_seconds.count() > 2)
                {
                    time_start = std::chrono::high_resolution_clock::now();
                    canopen::uploadSDO(id, DRIVERTEMPERATURE);
                    getErrors(id);
                    readManErrReg(id);
                }

                if (devices[id].getInitialized())
                {
                    devices[id].updateDesiredPos();
                    sendPos((uint16_t)id, (double)devices[id].getDesiredPos());
                }
            }
            canopen::sendSync();
            std::this_thread::sleep_for(syncInterval - (std::chrono::high_resolution_clock::now() - tic ));
        }

    }
}


std::function< void (uint16_t CANid, double velocityValue) > sendVel;
std::function< void (uint16_t CANid, double positionValue) > sendPos;
std::function< void (uint16_t CANid, double positionValue, double velocityValue) > sendPosPPMode;

//!Function to write the value of the position for operating in IP mode
/*! The function takes 2 arguments 
 * CANid is the CAN ID of the node
 * positionValue is the value of the next position in double*/
void defaultPDOOutgoing_interpolated(uint16_t CANid, double positionValue)
{
    static const uint16_t myControlword = (CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_ENABLE_IP_MODE);
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = 0x300 + CANid;
    msg.MSGTYPE = 0x00;
    msg.LEN = 4;
    int32_t mdegPos = rad2mdeg(positionValue);
    msg.DATA[0] = mdegPos & 0xFF;
    msg.DATA[1] = (mdegPos >> 8) & 0xFF;
    msg.DATA[2] = (mdegPos >> 16) & 0xFF;
    msg.DATA[3] = (mdegPos >> 24) & 0xFF;
    CAN_Write(h, &msg);
}

//!Function to write the value of the position and also the control word
/*! The function takes 2 arguments 
 * CANid is the CAN ID of the node
 * positionValue is the value of the next position in double*/
void defaultPDOOutgoing(uint16_t CANid, double positionValue)
{
    static const uint16_t myControlword = (CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_ENABLE_IP_MODE);
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = 0x200 + CANid;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = myControlword & 0xFF;
    msg.DATA[1] = (myControlword >> 8) & 0xFF;
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    int32_t mdegPos = rad2mdeg(positionValue);
    msg.DATA[4] = mdegPos & 0xFF;
    msg.DATA[5] = (mdegPos >> 8) & 0xFF;
    msg.DATA[6] = (mdegPos >> 16) & 0xFF;
    msg.DATA[7] = (mdegPos >> 24) & 0xFF;
    CAN_Write(h, &msg);
}



//!Emergency signal
void defaultEMCY_incoming(uint16_t CANid, const TPCANRdMsg m)
{


    uint16_t mydata_low = m.Msg.DATA[0];
    uint16_t mydata_high = m.Msg.DATA[1];

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
    //! std::cout << "EMCY" << (uint16_t)CANid << " is: " << (uint16_t)m.Msg.DATA[0] << " "<< (uint16_t)m.Msg.DATA[1]<< " " << (uint16_t)m.Msg.DATA[2]<< " "<< (uint16_t)m.Msg.DATA[3]<< " "<< (uint16_t)m.Msg.DATA[4]<< " "<< (uint16_t)m.Msg.DATA[5]<< " "<< (uint16_t)m.Msg.DATA[6]<< " "<< (uint16_t)m.Msg.DATA[7]<< " "<< (uint16_t)m.Msg.DATA[8]<< std::endl;
=======
    //! std::cout << "EMCY" << (uint16_t)CANid << " is: " << (uint16_t)m.Msg.DATA[0] << std::endl;
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp


}

//!Function sets the various status variables
/*! The function takes 2 arguments
 * CANid is the CAN identifier of the device 
 * m is a TPCANRdMsg message as defined in libpcan.h */
void defaultPDO_incoming_status(uint16_t CANid, const TPCANRdMsg m)
{

    uint16_t mydata_low = m.Msg.DATA[0];
    uint16_t mydata_high = m.Msg.DATA[1];

    int8_t mode_display = m.Msg.DATA[2];
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
    
=======
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
	//!Not executed as use_limit_switch is set to FALSE always
    if(canopen::use_limit_switch)
    {


        uint16_t limit_switch_ = m.Msg.DATA[2];

        bool hardware_limit_positive = limit_switch_ & 0x02;
        bool hardware_limit_negative = limit_switch_ & 0x01;

        devices[CANid].setNegativeLimit(hardware_limit_negative);
        devices[CANid].setPositiveLimit(hardware_limit_positive);
    }
	
	//! Read the status variables that correspond to the bits in mydata_low
	bool ready_switch_on = mydata_low & 0x01;
    bool switched_on = mydata_low & 0x02;
    bool op_enable = mydata_low & 0x04;
    bool fault = mydata_low & 0x08;
    //! std::cout << "fault PDO" << fault << std::endl;
    bool volt_enable = mydata_low & 0x10;
    bool quick_stop = mydata_low & 0x20;
    bool switch_on_disabled = mydata_low & 0x40;
    bool warning = mydata_low & 0x80;

	//! Read the status variables that correspond to the bits in mydata_high
    bool mode_specific = mydata_high & 0x01;
    bool remote = mydata_high & 0x02;
    bool target_reached = mydata_high & 0x04;
    bool internal_limit = mydata_high & 0x08;
    bool op_specific = mydata_high & 0x10;
    bool op_specific1 = mydata_high & 0x20;
    bool man_specific1 = mydata_high & 0x40;
    bool man_specific2 = mydata_high & 0x80;

	//! Set ip mode status bit depending on the status bits updated as above
    bool ip_mode = ready_switch_on & switched_on & op_enable & volt_enable;

	//!Set motor state variable depending on the status variables set
    if(!ready_switch_on)
    {
        if(fault)
        {
            devices[CANid].setMotorState(canopen::MS_FAULT);
        }
        else if(switch_on_disabled)
        {
            devices[CANid].setMotorState(canopen::MS_SWITCHED_ON_DISABLED);
        }
        else
            devices[CANid].setMotorState(canopen::MS_NOT_READY_TO_SWITCH_ON);
    }

    else
    {
        if(switched_on)
        {
            if(op_enable)
            {

                //!if(volt_enable)
                //! {
                devices[CANid].setMotorState(canopen::MS_OPERATION_ENABLED);
                //! }

            }
            else
                devices[CANid].setMotorState(canopen::MS_SWITCHED_ON);
        }
        else if(!quick_stop)
            devices[CANid].setMotorState(canopen::MS_QUICK_STOP_ACTIVE);

        else
            devices[CANid].setMotorState(canopen::MS_READY_TO_SWITCH_ON);

    }

    if(fault & op_enable & switched_on & ready_switch_on)
        devices[CANid].setMotorState(canopen::MS_FAULT_REACTION_ACTIVE);

	//!Set status variables of the device with the given CANId by using the setter functions of the canopen.h
    devices[CANid].setFault(fault);
    devices[CANid].setIPMode(ip_mode);
    devices[CANid].setHoming(op_specific);
    devices[CANid].setOpSpec0(op_specific);
    devices[CANid].setOpSpec1(op_specific1);
    devices[CANid].setManSpec1(man_specific1);
    devices[CANid].setManSpec2(man_specific2);
    devices[CANid].setInternalLimits(internal_limit);
    devices[CANid].setTargetReached(target_reached);
    devices[CANid].setRemote(remote);
    devices[CANid].setModeSpec(mode_specific);
    devices[CANid].setWarning(warning);
    devices[CANid].setSwitchOnDisable(switch_on_disabled);
    devices[CANid].setQuickStop(quick_stop);
    devices[CANid].setOpEnable(op_enable);
    devices[CANid].setVoltageEnabled(volt_enable);
    devices[CANid].setReadySwitchON(ready_switch_on);
    devices[CANid].setSwitchON(switched_on);

    devices[CANid].setCurrentModeofOperation(mode_display);

    //! std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;
}

//!Funcion to set the desired position,actual velocity, Actual position and Time stamps from incoming PDO message
/*! The function takes 2 arguments
 * CANid is the CAN identifier of the device 
 * m is a TPCANRdMsg message as defined in libpcan.h */
void defaultPDO_incoming_pos(uint16_t CANid, const TPCANRdMsg m)
{
    double newPos = mdeg2rad(m.Msg.DATA[0] + (m.Msg.DATA[1] << 8) + (m.Msg.DATA[2] << 16) + (m.Msg.DATA[3] << 24));
    double newVel = mdeg2rad(m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24));

    //!newPos = devices[CANid].getConversionFactor()*newPos; //TODO: conversion from yaml file
    //!newVel = devices[CANid].getConversionFactor()*newVel;

    if (devices[CANid].getTimeStamp_msec() != std::chrono::milliseconds(0) || devices[CANid].getTimeStamp_usec() != std::chrono::microseconds(0))
    {
        auto deltaTime_msec = std::chrono::milliseconds(m.dwTime) - devices[CANid].getTimeStamp_msec();
        auto deltaTime_usec = std::chrono::microseconds(m.wUsec) - devices[CANid].getTimeStamp_usec();
        double deltaTime_double = static_cast<double>(deltaTime_msec.count()*1000 + deltaTime_usec.count()) * 0.000001;
        double result = (newPos - devices[CANid].getActualPos()) / deltaTime_double;
        devices[CANid].setActualVel(result);
        if (! devices[CANid].getInitialized())
        {
            devices[CANid].setDesiredPos(newPos);
        }
        //!std::cout << "actualPos: " << devices[CANid].getActualPos() << "  desiredPos: " << devices[CANid].getDesiredPos() << std::endl;
    }

    devices[CANid].setActualPos(newPos);
    //!devices[CANid].setActualVel(newVel);

    devices[CANid].setTimeStamp_msec(std::chrono::milliseconds(m.dwTime));
    devices[CANid].setTimeStamp_usec(std::chrono::microseconds(m.wUsec));

}

//!Function to read the position value and the time stamp from the incoming PDO message 
/*! The function takes 2 arguments
 * CANid is the CAN identifier of the device 
 * m is a TPCANRdMsg message as defined in libpcan.h */
void defaultPDO_incoming(uint16_t CANid, const TPCANRdMsg m)
{
    double newPos = mdeg2rad(m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24) );

    if (devices[CANid].getTimeStamp_msec() != std::chrono::milliseconds(0) || devices[CANid].getTimeStamp_usec() != std::chrono::microseconds(0))
    {
        auto deltaTime_msec = std::chrono::milliseconds(m.dwTime) - devices[CANid].getTimeStamp_msec();
        auto deltaTime_usec = std::chrono::microseconds(m.wUsec) - devices[CANid].getTimeStamp_usec();
        double deltaTime_double = static_cast<double>(deltaTime_msec.count()*1000 + deltaTime_usec.count()) * 0.000001;
        double result = (newPos - devices[CANid].getActualPos()) / deltaTime_double;
        devices[CANid].setActualVel(result);
        if (! devices[CANid].getInitialized())
        {
            devices[CANid].setDesiredPos(newPos);
        }
        //!std::cout << "actualPos: " << devices[CANid].getActualPos() << "  desiredPos: " << devices[CANid].getDesiredPos() << std::endl;
    }


    devices[CANid].setActualPos(newPos);
    devices[CANid].setTimeStamp_msec(std::chrono::milliseconds(m.dwTime));
    devices[CANid].setTimeStamp_usec(std::chrono::microseconds(m.wUsec));

    uint16_t mydata_low = m.Msg.DATA[0];
    uint16_t mydata_high = m.Msg.DATA[1];
	
	//!Set the various status message depending on the received PDO message
    bool ready_switch_on = mydata_low & 0x01;
    bool switched_on = mydata_low & 0x02;
    bool op_enable = mydata_low & 0x04;
    bool fault = mydata_low & 0x08;
    //! std::cout << "fault PDO" << fault << std::endl;
    bool volt_enable = mydata_low & 0x10;
    bool quick_stop = mydata_low & 0x20;
    bool switch_on_disabled = mydata_low & 0x40;
    bool warning = mydata_low & 0x80;

    bool mode_specific = mydata_high & 0x01;
    bool remote = mydata_high & 0x02;
    bool target_reached = mydata_high & 0x04;
    bool internal_limit = mydata_high & 0x08;
    bool op_specific = mydata_high & 0x10;
    bool op_specific1 = mydata_high & 0x20;
    bool man_specific1 = mydata_high & 0x40;
    bool man_specific2 = mydata_high & 0x80;

    bool ip_mode = ready_switch_on & switched_on & op_enable & volt_enable;

	//!Set motor state variable depending on the status variables set
    if(!ready_switch_on)
    {
        if(fault)
        {
            devices[CANid].setMotorState(canopen::MS_FAULT);
        }
        else if(switch_on_disabled)
        {
            devices[CANid].setMotorState(canopen::MS_SWITCHED_ON_DISABLED);
        }
        else
            devices[CANid].setMotorState(canopen::MS_NOT_READY_TO_SWITCH_ON);
    }

    else
    {
        if(switched_on)
        {
            if(op_enable)
            {

                //!if(volt_enable)
                //! {
                devices[CANid].setMotorState(canopen::MS_OPERATION_ENABLED);
                //! }

            }
            else
                devices[CANid].setMotorState(canopen::MS_SWITCHED_ON);
        }
        else if(!quick_stop)
            devices[CANid].setMotorState(canopen::MS_QUICK_STOP_ACTIVE);

        else
            devices[CANid].setMotorState(canopen::MS_READY_TO_SWITCH_ON);

    }

    if(fault & op_enable & switched_on & ready_switch_on)
        devices[CANid].setMotorState(canopen::MS_FAULT_REACTION_ACTIVE);

	//!Set status variables of the device with the given CANId by using the setter functions of the canopen.h
    devices[CANid].setFault(fault);
    devices[CANid].setIPMode(ip_mode);
    devices[CANid].setHoming(op_specific);
    devices[CANid].setOpSpec0(op_specific);
    devices[CANid].setOpSpec1(op_specific1);
    devices[CANid].setManSpec1(man_specific1);
    devices[CANid].setManSpec2(man_specific2);
    devices[CANid].setInternalLimits(internal_limit);
    devices[CANid].setTargetReached(target_reached);
    devices[CANid].setRemote(remote);
    devices[CANid].setModeSpec(mode_specific);
    devices[CANid].setWarning(warning);
    devices[CANid].setSwitchOnDisable(switch_on_disabled);
    devices[CANid].setQuickStop(quick_stop);
    devices[CANid].setOpEnable(op_enable);
    devices[CANid].setVoltageEnabled(volt_enable);
    devices[CANid].setReadySwitchON(ready_switch_on);
    devices[CANid].setSwitchON(switched_on);

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
    //! std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;
=======
    //!std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp


}

/***************************************************************/
//!		define functions for receiving data
/***************************************************************/

//!  Function that spawns a thread that runs using the associated listener function
void initListenerThread(std::function<void ()> const& listener)
{
    std::thread listener_thread(listener);
    listener_thread.detach();
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
=======
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
    //!std::cout << "Listener thread initialized" << std::endl;
}

//! Function for the default listener thread
/*! The function takes no arguments and returns nothing */
void defaultListener()
{
    while(true)
    {
        //!std::cout << "Reading incoming data" << std::endl;
        TPCANRdMsg m;
        errno = LINUX_CAN_Read(h, &m);
        if (errno)
            perror("LINUX_CAN_Read() error");

        //! incoming SYNC
        else if (m.Msg.ID == 0x080)
        {
            //! std::cout << std::hex << "SYNC received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
        }

        //! incoming EMCY
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
        else if (m.Msg.ID >= 0x081 && m.Msg.ID <= 0x0FF)
        {
        //!   std::cout << std::hex << "EMCY received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
          if (incomingEMCYHandlers.find(m.Msg.ID) != incomingEMCYHandlers.end())
             incomingEMCYHandlers[m.Msg.ID](m);
        }
=======
        /*!{
         //!   std::cout << std::hex << "EMCY received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
          //! if (incomingEMCYHandlers.find(m.Msg.ID) != incomingEMCYHandlers.end())
           //!     incomingEMCYHandlers[m.Msg.ID](m);
        }*/
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp

        //! incoming TIME
        else if (m.Msg.ID == 0x100)
        {
            //! std::cout << std::hex << "TIME received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
        }

        //! incoming PD0
        else if (m.Msg.ID >= 0x180 && m.Msg.ID <= 0x4FF)
        {
            /*!std::cout << std::hex << "PDO received:  " << (m.Msg.ID - 0x180) << "  " << m.Msg.DATA[0] << " " << m.Msg.DATA[1] << " " << m.Msg.DATA[2] << " " << m.Msg.DATA[3] << " " << m.Msg.DATA[4] << " " << m.Msg.DATA[5] << " " << m.Msg.DATA[6] << " " <<  m.Msg.DATA[7] << " " << std::endl;
            std::cout << std::hex << "PDO received:  " << (uint16_t)(m.Msg.ID - 0x180) << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " <<  (uint16_t)m.Msg.DATA[7] << " " << std::endl;*/
            if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end())
                incomingPDOHandlers[m.Msg.ID](m);
        }

        //! incoming SD0
        else if (m.Msg.ID >= 0x580 && m.Msg.ID <= 0x5FF)
        {
            //!std::cout << std::hex << "SDO received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
            SDOkey sdoKey(m);
            //!Copy the contents of DATA bytes of incoming PDO message to protect_msg if sdo_protect is set to 1
            if(sdo_protect)
            {
                std::copy(std::begin(m.Msg.DATA), std::end(m.Msg.DATA), std::begin(protect_msg));
                sdo_protect = false;
            }
            else
            {
                if (incomingErrorHandlers.find(sdoKey) != incomingErrorHandlers.end())
                    incomingErrorHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
                else if (incomingDataHandlers.find(sdoKey) != incomingDataHandlers.end())
                    incomingDataHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
                else if (incomingManufacturerDetails.find(sdoKey) != incomingManufacturerDetails.end())
                    incomingManufacturerDetails[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
            }
        }

        //! incoming NMT error control 
        else if (m.Msg.ID >= 0x700 && m.Msg.ID <= 0x7FF)
        {
            //std::cout << std::hex << "NMT received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << std::endl;
            uint16_t CANid = (uint16_t)(m.Msg.ID - 0x700);
            
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
            std::cout << "Bootup received. Node-ID =  " << CANid << std::endl;
            std::map<uint8_t,Device>::const_iterator search = devices.find(CANid);
            if(search != devices.end())
            {
                std::cout << "Found " << (u_int16_t)search->first << "\n";
                devices[CANid].setNMTInit(true);
=======
            if (m.Msg.DATA[0] == 0x00)
            {          
                std::cout << "Bootup received. Node-ID =  " << CANid << std::endl;
                std::map<uint8_t,Device>::const_iterator search = devices.find(CANid);
                if(search != devices.end())
                {
                        std::cout << "Found " << (u_int16_t)search->first << "\n";
                        std::cout << "Initializing..." << "\n";
                        devices[CANid].setNMTInit(true);
                }
                else
                {
                       std::cout << "Not found" << std::endl;
                       std::cout << "Ignoring" << std::endl;
                }

>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
            }
            else
            {
                std::cout << "Node:" << CANid << " could not be found on the required devices list." << std::endl;
                std::cout << "Ignoring" << std::endl;
            }

        }
        else
        {
            //!std::cout << "Received unknown message" << std::endl;
        }
    }
}

/******************************************************************************
 * Define get errors function
 *****************************************************************************/

//! Function to get the errorword from a device with given CANId
/*! The function takes 1 parameter
 * CANid is the CAN identifier of the device whose errorword is be read */
void getErrors(uint16_t CANid)
{
    canopen::uploadSDO(CANid, canopen::ERRORWORD);
}

//! Function to get the manufacturer name and to set the same using setter function
void manufacturer_incoming(uint8_t CANid, BYTE data[8])
{
    sdo_protect = true;

    if(data[1]+(data[2]<<8) == 0x1008)
    {
        std::vector<char> manufacturer_device_name = canopen::obtainManDevName(CANid, data[4]);

        devices[CANid].setManufacturerDevName(manufacturer_device_name);
    }
    /*!
    else if(data[1]+(data[2]<<8) == 0x1009)
    {

    }
    */
}

//! Function to read the incoming error register or the manufacturer error register from incoming data bytes and to set the same information using setter function
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/*! The function takes 2 inputs
 *  CANid is the id of the CAN device
 * data[8] is  the 8 byte data */
=======
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
void errorword_incoming(uint8_t CANid, BYTE data[8])
{
    std::stringstream str_stream;

    if(data[1]+(data[2]<<8) == 0x1001)
    {
        uint16_t error_register;
        error_register = data[4];

        str_stream << "error_register=0x" << std::hex << (int)error_register << ", categories:";

        if ( error_register & canopen::EMC_k_1001_GENERIC )
            str_stream << " generic,";
        if ( error_register & canopen::EMC_k_1001_CURRENT)
            str_stream << " current,";
        if ( error_register & canopen::EMC_k_1001_VOLTAGE )
            str_stream << " voltage,";
        if ( error_register & canopen::EMC_k_1001_TEMPERATURE )
            str_stream << " temperature,";
        if ( error_register & canopen::EMC_k_1001_COMMUNICATION )
            str_stream << " communication,";
        if ( error_register & canopen::EMC_k_1001_DEV_PROF_SPEC )
            str_stream << " device profile specific,";
        if ( error_register & canopen::EMC_k_1001_RESERVED )
            str_stream << " reserved,";
        if ( error_register & canopen::EMC_k_1001_MANUFACTURER)
            str_stream << " manufacturer specific";
        str_stream << "\n";

        devices[CANid].setErrorRegister(str_stream.str());
    }
    else if(data[1]+(data[2]<<8) == 0x1002)
    {
        uint16_t code = data[4];
        uint16_t classification = data[5];

        str_stream << "manufacturer_status_register=0x" << std::hex << int(classification) << int(code) <<
                      ": code=0x" << std::hex << int( code ) << " (" << errorsCode[int(code)] << "),"
                   << ", classification=0x" << std::hex << int( classification ) << std::dec;
        if ( classification == 0x88 )
            str_stream << " (CMD_ERROR)";
        if ( classification == 0x89 )
            str_stream << " (CMD_WARNING)";
        if ( classification == 0x8a )
            str_stream << " (CMD_INFO)";
        str_stream << "\n";

        devices[CANid].setManufacturerErrorRegister(str_stream.str());
    }
}

//! Function to read the Manufacturer Error Register of the device with the given CANId
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
 /*! It takes CANid of the device as the input*/
=======
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
void readManErrReg(uint16_t CANid)
{
    canopen::uploadSDO(CANid, canopen::MANUFACTURER);
}

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
//! Function to read the Manufacturer Error Register of the device with the given CANId
 /*! The function takes 2 inputs 
  * CANid of the device 
  *  m is the shared pointer of type TPCANRdMsg */
=======

>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
void readErrorsRegister(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    canopen::uploadSDO(CANid, canopen::STATUSWORD);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen::processSingleSDO(CANid, m);

    canopen::uploadSDO(CANid, canopen::ERRORWORD);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen::processSingleSDO(CANid, m);

    uint16_t error_register;
    error_register = m->Msg.DATA[4];

    std::cout << "error_register=0x" << std::hex << (int)error_register << ", categories:";

    if ( error_register & canopen::EMC_k_1001_GENERIC )
        std::cout << " generic,";
    if ( error_register & canopen::EMC_k_1001_CURRENT)
        std::cout << " current,";
    if ( error_register & canopen::EMC_k_1001_VOLTAGE )
        std::cout << " voltage,";
    if ( error_register & canopen::EMC_k_1001_TEMPERATURE )
        std::cout << " temperature,";
    if ( error_register & canopen::EMC_k_1001_COMMUNICATION )
        std::cout << " communication,";
    if ( error_register & canopen::EMC_k_1001_DEV_PROF_SPEC )
        std::cout << " device profile specific,";
    if ( error_register & canopen::EMC_k_1001_RESERVED )
        std::cout << " reserved,";
    if ( error_register & canopen::EMC_k_1001_MANUFACTURER)
        std::cout << " manufacturer specific";
    std::cout << "\n";
}

//! Function to read the Vendor ID 
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
 /*! It takes CANid of the device as the input*/
=======
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
std::vector<uint16_t> obtainVendorID(uint16_t CANid)
{
    canopen::uploadSDO(CANid, canopen::IDENTITYVENDORID);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

//! Function to obtain the Product Code and store it in the array 'product_code'
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/*! The function takes 2 inputs 
  * CANid of the device 
  *  m is the shared pointer of type TPCANRdMsg */
=======
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
std::vector<uint16_t> obtainProdCode(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    canopen::uploadSDO(CANid, canopen::IDENTITYPRODUCTCODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<uint16_t> product_code;

    canopen::processSingleSDO(CANid, m);

    uint16_t id4 = m->Msg.DATA[4];
    uint16_t id3 = m->Msg.DATA[5];
    uint16_t id2 = m->Msg.DATA[6];
    uint16_t id1 = m->Msg.DATA[7];

    product_code.push_back(id1);
    product_code.push_back(id2);
    product_code.push_back(id3);
    product_code.push_back(id4);

    return product_code;

}

//! Function to obtain the Revision Number and store it in rev_number
/*!The function takes 2 variables 
 * CANid is the CAN ID of the device
 * m is a object of type TPCANRdMsg accessed by a shared pointer */
uint16_t obtainRevNr(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    canopen::uploadSDO(CANid, canopen::IDENTITYREVNUMBER);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));


    canopen::processSingleSDO(CANid, m);

    uint16_t rev_number = m->Msg.DATA[4];

    return rev_number;

}

//! Function to obtain the Manufacturer Name and store it in rev_number
/*!The function takes 2 variables 
 * CANid is the CAN ID of the device
 * size_name is the integer variable to store the revision number */
std::vector<char> obtainManDevName(uint16_t CANid, int size_name)
{

    std::vector<char> manufacturer_device_name;

    canopen::requestDataBlock1(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (auto it : protect_msg)
    {
        if(manufacturer_device_name.size() <= size_name)
            manufacturer_device_name.push_back(it);
    }

    return manufacturer_device_name;

}



//! Function to obtain the Manufacturer Hardware Version and store it in manufacturer_hw_version
/*!The function takes 2 variables 
 * CANid is the CAN ID of the device
 * m is a object of type TPCANRdMsg accessed by a shared pointer */
std::vector<char> obtainManHWVersion(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    canopen::uploadSDO(CANid, canopen::MANUFACTURERHWVERSION);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<char> manufacturer_hw_version;

    canopen::processSingleSDO(CANid, m);

    int size = m->Msg.DATA[4];

    canopen::requestDataBlock1(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_hw_version.size() <= size)
            manufacturer_hw_version.push_back(it);
    }


    canopen::requestDataBlock2(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_hw_version.size() <= size)
            manufacturer_hw_version.push_back(it);
    }

    return manufacturer_hw_version;
}

//! Function to obtain the Manufacturer Software Version and store it in manufacturer_sw_version
/*!The function takes 2 variables 
 * CANid is the CAN ID of the device
 * m is a object of type TPCANRdMsg accessed by a shared pointer */
std::vector<char> obtainManSWVersion(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    std::vector<char> manufacturer_sw_version;

    canopen::uploadSDO(CANid, canopen::MANUFACTURERSOFTWAREVERSION);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);

    int size = (uint8_t)m->Msg.DATA[4];

    canopen::requestDataBlock1(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }


    canopen::requestDataBlock2(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }

    canopen::requestDataBlock1(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }

    canopen::requestDataBlock2(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }

    return manufacturer_sw_version;

}


//!Function to evaluate the incoming SDO Status word
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/*! The function takes 2 inputs
 *  CANid is the id of the CAN device
 * data[8] is  the 8 byte data */
=======
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
void sdo_incoming(uint8_t CANid, BYTE data[8])
{
    uint16_t SDOid = data[1]+(data[2]<<8);

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
    if(SDOid == STATUSWORD.index) //!The incoming message is a result from a statusWord Request
    {
=======
    if(data[1]+(data[2]<<8) == 0x6041)
    {
		//!The incoming message is a result from a statusWord Request
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
        uint16_t mydata_low = data[4];
        uint16_t mydata_high = data[5];
		
		//!Update the various status bits
        bool ready_switch_on = mydata_low & 0x01;
        bool switched_on = mydata_low & 0x02;
        bool op_enable = mydata_low & 0x04;
        bool fault = mydata_low & 0x08;
        bool volt_enable = mydata_low & 0x10;
        bool quick_stop = mydata_low & 0x20;
        bool switch_on_disabled = mydata_low & 0x40;
        bool warning = mydata_low & 0x80;

        bool mode_specific = mydata_high & 0x01;
        bool remote = mydata_high & 0x02;
        bool target_reached = mydata_high & 0x04;
        bool internal_limit = mydata_high & 0x08;
        bool op_specific = mydata_high & 0x10;
        bool op_specific1 = mydata_high & 0x20;
        bool man_specific1 = mydata_high & 0x40;
        bool man_specific2 = mydata_high & 0x80;

		//!Update information about the Interpolation mode
        bool ip_mode = ready_switch_on & switched_on & op_enable & volt_enable;

		//!Set the motor state variable
        if(!ready_switch_on)
        {
            if(fault)
            {
                devices[CANid].setMotorState(canopen::MS_FAULT);
            }
            else if(switch_on_disabled)
            {
                devices[CANid].setMotorState(canopen::MS_SWITCHED_ON_DISABLED);
            }
            else
                devices[CANid].setMotorState(canopen::MS_NOT_READY_TO_SWITCH_ON);
        }

        else
        {
            if(switched_on)
            {
                if(op_enable)
                {

                    //!if(volt_enable)
                    //! {
                    devices[CANid].setMotorState(canopen::MS_OPERATION_ENABLED);
                    //! }

                }
                else
                    devices[CANid].setMotorState(canopen::MS_SWITCHED_ON);
            }
            else if(!quick_stop)
                devices[CANid].setMotorState(canopen::MS_QUICK_STOP_ACTIVE);

            else
                devices[CANid].setMotorState(canopen::MS_READY_TO_SWITCH_ON);

        }

        if(fault & op_enable & switched_on & ready_switch_on)
            devices[CANid].setMotorState(canopen::MS_FAULT_REACTION_ACTIVE);


		//!Set the various status bits using setter fumction
        devices[CANid].setFault(fault);
        devices[CANid].setHoming(op_specific);
        devices[CANid].setOpSpec0(op_specific);
        devices[CANid].setOpSpec1(op_specific1);
        devices[CANid].setManSpec1(man_specific1);
        devices[CANid].setManSpec2(man_specific2);
        devices[CANid].setInternalLimits(internal_limit);
        devices[CANid].setTargetReached(target_reached);
        devices[CANid].setRemote(remote);
        devices[CANid].setModeSpec(mode_specific);
        devices[CANid].setWarning(warning);
        devices[CANid].setSwitchOnDisable(switch_on_disabled);
        devices[CANid].setQuickStop(quick_stop);
        devices[CANid].setOpEnable(op_enable);
        devices[CANid].setVoltageEnabled(volt_enable);
        devices[CANid].setReadySwitchON(ready_switch_on);
        devices[CANid].setSwitchON(switched_on);

        //!std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;
    }
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
    else if(SDOid == DRIVERTEMPERATURE.index) //!This is a result from a temperature register request
    {
        devices[CANid].setDriverTemperature(data[4]);
    }
    else if(SDOid == MODES_OF_OPERATION_DISPLAY.index) //!Incoming message is a mode of operation display
    {
        devices[CANid].setCurrentModeofOperation(data[4]);
=======

    else if(data[1]+(data[2]<<8) == 0x22A2)
    { //!This is a result from a temperature register request
        devices[CANid].setDriverTemperature(data[4]);
    }

    else if(data[1]+(data[2]<<8) == 0x6061)
    { //!Incoming message is a mode of operation display
         devices[CANid].setCurrentModeofOperation(data[4]);
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
    }

}

//!FUnction to process the SDO
/*!The function takes 2 variables 
 * CANid is the CAN ID of the device
 * message is a object of type TPCANRdMsg accessed by a shared pointer */
void processSingleSDO(uint8_t CANid, std::shared_ptr<TPCANRdMsg> message)
{
    message->Msg.ID = 0x00;

    while (message->Msg.ID != (0x580+CANid))
    {
        LINUX_CAN_Read(canopen::h, message.get());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
//! Function to confirm that the PDO mapping was changed
/*! Function takes chain name as an argument and returns nothing */

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
void pdoChanged(std::string chainName)
=======
//! Function to confirm that the PDO mapping was changed
/*! Function takes no arguments and returns nothing */
void pdoChanged()
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        TPCANMsg* mes;
        //////////////////// Enable tpdo4
        mes->ID =id + 0x600;
        mes->MSGTYPE = 0x00;
        mes->LEN = 8;
        mes->DATA[0] = 0x2F;
        mes->DATA[1] = 0x20;
        mes->DATA[2] = 0x2F;
        mes->DATA[3] = 0x04;
        mes->DATA[4] = 0x00;
        mes->DATA[5] = 0x00;
        mes->DATA[6] = 0x00;
        mes->DATA[7] = 0x01;
        CAN_Write(canopen::h, mes);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
//! FUnction to disable a Receive PDO
/*! Function takes the chain name and number of the object in the RPDO that has to be disabled and diables the corresponding object in the RPDO.
 * This function does not return any value */
void disableRPDO(std::string chainName, int object)
=======

//! FUnction to disable a Receive PDO
/*! Function takes the number of the object in the RPDO that has to be disabled and diables the corresponding object in the RPDO.
 * This function does not return any value */
void disableRPDO(int object)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        if(object == 0)
        {
            int32_t data = (canopen::RPDO1_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::RPDO2_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);

        }

        else if(object == 2)
        {
            int32_t data = (canopen::RPDO3_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }

        else if(object == 3)

        {
            int32_t data = (canopen::RPDO4_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /////////////////////////

    }
}

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
void setObjects(std::string chainName)
=======
//!Function to set some SDO objects ((0x6081,0x00),(0x607f,0x00),(0x6083,0x00),(0x60c5,0x00),(0x60c6,0x00),(0x6082,0x00)) to the value 0x00001388
/*! (0x6081,0x00) - Profile velocity
 * (0x607f,0x00) - Max. Profile velocity
 * (0x6083,0x00) - Profile acceleration
 * (0x60c5,0x00) - Max. Acceleration
 * (0x60c6,0x00) - Max. Deceleration
 * (0x6082,0x00) - End velocity
/* The function takes no arguments and returns nothing*/
void setObjects()
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        int32_t data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x6081,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x607f,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x6083,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x60c5,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x60c6,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x6082,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}

//!Function to clear the existing Receive PDO mapping
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/*! Function takes the chain name and number of the object in the RPDO that has to be cleared and clears the corresponding object in the RPDO.
 * This function does not return any value */
void clearRPDOMapping(std::string chainName, int object)
=======
/*! Function takes the number of the object in the RPDO that has to be cleared and clears the corresponding object in the RPDO.
 * This function does not return any value */
void clearRPDOMapping(int object)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        int32_t data = (0x00 << 16) + (0x80 << 24);

        sendSDO_unknown(id, SDOkey(RPDO_map.index+object,0x00), data);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

//!Function to make a new Receive PDO mapping
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/*! the function takes 5 variables
 * chainName is the name of the chain
=======
/*! the function takes 4 variables
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
 * object is the number of the object in RPDO for which a new mapping is formed
 * registers is a array of string which contains the register in which the new mapping has to be done 
 * sizes is a array of integers which contains the size of register in which the new mapping has to be done
 * sync_type is an unsigned integer */
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
void makeRPDOMapping(std::string chainName, int object, std::vector<std::string> registers, std::vector<int> sizes , u_int8_t sync_type)
=======
void makeRPDOMapping(int object, std::vector<std::string> registers, std::vector<int> sizes , u_int8_t sync_type)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        int ext_counter=0;
        for(int counter=0; counter < registers.size();counter++)
        {
            /////////////////////////
            int index_data;

            std::stringstream str_stream;
            str_stream << registers[counter];
            str_stream >> std::hex >> index_data;

            str_stream.str( std::string() );
            str_stream.clear();

            /////////////////////////
            /// \brief data
            ///
            int32_t data = (sizes[counter]) + (index_data << 8);

            sendSDO(id, SDOkey(RPDO_map.index+object,counter+1), data);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            ext_counter++;
        }
        /////////////////////////
        //////////////////// ASync

        sendSDO(id, SDOkey(RPDO.index+object,0x02), u_int8_t(sync_type));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //////////////////////
        ///
        ///
        /////////////////////// Mapping x objects
        sendSDO(id, SDOkey(RPDO_map.index+object,0x00), u_int8_t(ext_counter));

    }
}

//! Function to enable a newly set Receive PDO
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/*! Function takes the chain name and number of the object in the RPDO that has to be enabled and enables the corresponding object in the RPDO.
 * This function does not return any value */
void enableRPDO(std::string chainName, int object)
=======
/*! Function takes the number of the object in the RPDO that has to be enabled and enables the corresponding object in the RPDO.
 * This function does not return any value */
void enableRPDO(int object)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        if(object ==0)
        {
            int32_t data = (canopen::RPDO1_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::RPDO2_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 2)
        {
            int32_t data = (canopen::RPDO3_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 3)
        {
            int32_t data = (canopen::RPDO4_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /////////////////////////
    }
}


/*****************************
 *
 * Mapping for PDO1
 **/

//! Function to disable a transmit PDO
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/*! Function takes the chain name and number of the object in the TPDO that has to be disabled and disables the corresponding object in the TPDO.
 * This function does not return any value */
void disableTPDO(std::string chainName,int object)
=======
/*! Function takes the number of the object in the TPDO that has to be disabled and disables the corresponding object in the TPDO.
 * This function does not return any value */
void disableTPDO(int object)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{

    for(auto id : canopen::deviceGroups[chainName].getCANids())
    {

        //////////////////// Disable tpdo4

        if(object == 0)
        {
            int32_t data = (canopen::TPDO1_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::TPDO2_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);

        }

        else if(object == 2)
        {
            int32_t data = (canopen::TPDO3_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }

        else if(object == 3)

        {
            int32_t data = (canopen::TPDO4_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }

        else
            std::cout << "Incorrect object for mapping" << std::endl;



        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /////////////////////////
    }


}

//! Function to clear an existing Transmit PDO mapping
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/*! Function takes the chain name and number of the object in the TPDO that has to be cleared and clears the corresponding object in the TPDO.
 * This function does not return any value */
void clearTPDOMapping(std::string chainName, int object)
=======
/*! Function takes the number of the object in the TPDO that has to be cleared and clears the corresponding object in the TPDO.
 * This function does not return any value */
void clearTPDOMapping(int object)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        //////////////////// clear mapping
        ///
        //int32_t data = (0x00 << 16) + (0x00 << 24);
        sendSDO(id, SDOkey(TPDO_map.index+object,0x00), u_int8_t(0x00));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

//! Function to make a new Transmit PDO mapping
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/*! the function takes 5 variables
 * chainName is the name of the chain 
=======
/*! the function takes 4 variables
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
 * object is the number of the object in TPDO for which a new mapping is formed
 * registers is a array of string which contains the register in which the new mapping has to be done 
 * sizes is a array of integers which contains the size of register in which the new mapping has to be done
 * sync_type is an unsigned integer */
<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
void makeTPDOMapping(std::string chainName, int object, std::vector<std::string> registers, std::vector<int> sizes, u_int8_t sync_type)
=======
void makeTPDOMapping(int object, std::vector<std::string> registers, std::vector<int> sizes, u_int8_t sync_type)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        //////////////////// sub ind1=63
        ///
        ///
        ///
        int ext_counter=0;
        for(int counter=0; counter < registers.size();counter++)
        {
            /////////////////////////
            int index_data;

            std::stringstream str_stream;
            str_stream << registers[counter];
            str_stream >> std::hex >> index_data;

            str_stream.str( std::string() );
            str_stream.clear();

            /////////////////////////
            /// \brief data
            ///
            int32_t data = (sizes[counter]) + (index_data << 8);

            sendSDO(id, SDOkey(TPDO_map.index+object,counter+1), data);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            ext_counter++;
        }
        /////////////////////////
        //////////////////// ASync

        sendSDO(id, SDOkey(TPDO.index+object,0x02), u_int8_t(sync_type));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //////////////////////
        ///
        ///
        /////////////////////// Mapping x objects
        sendSDO(id, SDOkey(TPDO_map.index+object,0x00), u_int8_t(ext_counter));
    }

}

<<<<<<< HEAD:ipa_canopen_core/src/ipa_canopen_core.cpp
/! Function to enable the new Transmit PDO mapping
/*! Function takes the chain name and number of the object in the TPDO that has to be enabled and enables the corresponding object in the TPDO.
 * This function does not return any value */
void enableTPDO(std::string chainName, int object)
=======
//! Function to enable the new Transmit PDO mapping
/*! Function takes the number of the object in the TPDO that has to be enabled and enables the corresponding object in the TPDO.
 * This function does not return any value */
void enableTPDO(int object)
>>>>>>> origin:ipa_canopen_core/src/canopen.cpp
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        //////////////////// Enable tpdo4
        ///
        ///
        if(object ==0)
        {
            int32_t data = (canopen::TPDO1_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::TPDO2_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 2)
        {
            int32_t data = (canopen::TPDO3_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 3)
        {
            int32_t data = (canopen::TPDO4_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
    /////////////////////////
}




}

