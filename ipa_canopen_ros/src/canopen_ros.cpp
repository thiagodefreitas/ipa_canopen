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
 *   ROS package name: ipa_canopen_ros
 *
 * \author
 *   Author: Eduard Herkel, Thiago de Freitas, Tobias Sing
 * \author
 *   Supervised by: Eduard Herkel, Thiago de Freitas, Tobias Sing, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: December 2012
 *
 * \brief
 *   Implementation of canopen.
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
//!Including Required Header Files 

#include "ros/ros.h"
#include <urdf/model.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "brics_actuator/JointVelocities.h"
#include "cob_srvs/Trigger.h"
#include "cob_srvs/SetOperationMode.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <iostream>
#include <map>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <canopen.h>
#include <XmlRpcValue.h>
#include <JointLimits.h>

//!Declaration of 3 typedef for later use
typedef boost::function<bool(cob_srvs::Trigger::Request&, cob_srvs::Trigger::Response&)> TriggerType;
typedef boost::function<void(const brics_actuator::JointVelocities&)> JointVelocitiesType;
typedef boost::function<bool(cob_srvs::SetOperationMode::Request&, cob_srvs::SetOperationMode::Response&)> SetOperationModeCallbackType;

//!Definition of a struct named BusParams 
/*!Busparams has Baudrate and syncinterval as its members*/
struct BusParams
{
    std::string baudrate;
    uint32_t syncInterval;
};

//! Create a map which maps string to Busparams
std::map<std::string, BusParams> buses;

//!Declare a string named deviceFile
std::string deviceFile;

//! Declaration of a pointer
/*! joint_limits_ is a pointer of the type JointLimits */
JointLimits* joint_limits_;

//!Creating 2 vectors of type string 
std::vector<std::string> chainNames;
std::vector<std::string> jointNames;

//!Initialize the CAN bus connections for the devices in the chain

/*! The function takes 3 parameters and returns a boolean value
\param &req is a request of type cob_srvs::Trigger
\param &res is a response of type cob_srvs::Trigger
\param chainName is of the type string */
bool CANopenInit(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res, std::string chainName)
{

    canopen::init(deviceFile, canopen::syncInterval);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    for (auto device : canopen::devices)
    {
	//!How to send an SDO..? the format..?
        canopen::sendSDO(device.second.getCANid(), canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
        std::cout << "Setting IP mode for: " << (uint16_t)device.second.getCANid() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    //!Why is this required again.?
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    canopen::initDeviceManagerThread(canopen::deviceManager);

    for (auto device : canopen::devices)
    {
        device.second.setInitialized(true);
       /*! if(device.second.getHomingError())
          return false;*/

    }
    
    //!Is this a message..?
    res.success.data = true;
    res.error_message.data = "";
    
    //! Return true once the device is initialised 
    return true;
}

//! Recover devices that are set to Emergency state

/*! The function takes 3 parameters and returns a boolean value
\param &req is a request of type cob_srvs::Trigger
\param &res is a response of type cob_srvs::Trigger
\param chainName is of the type string */
bool CANopenRecover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res, std::string chainName)
{

    canopen::recover(deviceFile, canopen::syncInterval);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    for (auto device : canopen::devices)
    {
        canopen::sendSDO(device.second.getCANid(), canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
        std::cout << "Setting IP mode for: " << (uint16_t)device.second.getCANid() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    //!canopen::initDeviceManagerThread(canopen::deviceManager);

    for (auto device : canopen::devices)
    {
    	//!Restoring the Position and velocity to a value same as the previous value
        canopen::devices[device.second.getCANid()].setDesiredPos((double)device.second.getActualPos());
        canopen::devices[device.second.getCANid()].setDesiredVel(0);

        canopen::sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());
        canopen::sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());

        device.second.setInitialized(true);
    }

    res.success.data = true;
    res.error_message.data = "";
    
    //! Return a true value once the device is restored from an emergency state
    return true;
}

//!Halt the device in case of an emergency

/*! The function takes 3 parameters and returns a boolean value
\param &req is a request of type cob_srvs::Trigger
\param &res is a response of type cob_srvs::Trigger
\param chainName is of the type string */
bool CANOpenHalt(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res, std::string chainName)
{

    canopen::halt(deviceFile, canopen::syncInterval);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    res.success.data = true;
    res.error_message.data = "";
    
    //! Return a true value once the device is halted
    return true;
}

//!This is a dummy function. Not used elsewhere
bool setOperationModeCallback(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res, std::string chainName)
{
    res.success.data = true;  
    return true;
}

//!Set velocities and positions of the devices in the chain

/*!The function takes 2 parameters and returns nothing
\param msg is a message of the type brics_actuator::JointVelocities 
\param chainName is a string type */
void setVel(const brics_actuator::JointVelocities &msg, std::string chainName)
{
    if (!canopen::atFirstInit & !canopen::recover_active)
    {
    	//!Declare 2 vectors of the type double 
    	/*!velocities is to store the velocity of each device
    	positions is to store the position of each device */
        std::vector<double> velocities;
        std::vector<double> positions;


        for (auto it : msg.velocities)
        {
            velocities.push_back( it.value); //!Put all velocity values in the array velocities
        }

	int counter = 0;
       
        for (auto device : canopen::devices)
        {
		
            double pos = (double)device.second.getDesiredPos(); //! joint_limits_->getOffsets()[counter];
            positions.push_back(pos);//!Put all position values in the array positions
	    counter++;
        }
        
        //!What do these functions do.. Is it something like a check.?
        joint_limits_->checkVelocityLimits(velocities);
        joint_limits_->checkPositionLimits(velocities, positions);

        //! What is this..?
        canopen::deviceGroups[chainName].setVel(velocities);
    }
}

//!Read attributes from Parameter Server

/*! The function takes a ROS node as its parameter and returns nothing
\param n is a ROS Node */
void readParamsFromParameterServer(ros::NodeHandle n)
{
    XmlRpc::XmlRpcValue busParams;
    
    //!Checking for Parameters on the Parameter Server 
    if (!n.hasParam("devices") || !n.hasParam("chains"))
    {
        ROS_ERROR("Missing parameters on parameter server; shutting down node.");
        ROS_ERROR("Please consult the user manual for necessary parameter settings.");
        n.shutdown();
    }

    //!Getting the BaudRate and the SyncInterval for all devices
        n.getParam("devices", busParams);
    for (int i=0; i<busParams.size(); i++)
    {
        BusParams busParam;
        auto name = static_cast<std::string>(busParams[i]["name"]);
        busParam.baudrate = static_cast<std::string>(busParams[i]["baudrate"]);
        busParam.syncInterval = static_cast<int>(busParams[i]["sync_interval"]);
        buses[name] = busParam;
    }
    
    //!Get the chainnames and the corresponding jointnames, moduleIDs and devices in the chain
    XmlRpc::XmlRpcValue chainNames_XMLRPC;
    n.getParam("chains", chainNames_XMLRPC);
    
    //!Get all the chain names and put it into the array chainNames
     for (int i=0; i<chainNames_XMLRPC.size(); i++)
        chainNames.push_back(static_cast<std::string>(chainNames_XMLRPC[i]));
    
    //!For each chainname, get the corresponding jointnames, moduleIDs and device name
    for (auto chainName : chainNames) {
        XmlRpc::XmlRpcValue jointNames_XMLRPC;
        n.getParam("/" + chainName + "/joint_names", jointNames_XMLRPC);
	
	//! Put the joint names in a chain to array JointNames
	for (int i=0; i<jointNames_XMLRPC.size(); i++)
            jointNames.push_back(static_cast<std::string>(jointNames_XMLRPC[i]));
	
	//!Put the modulesIDs in a chain to array moduleIDs
	XmlRpc::XmlRpcValue moduleIDs_XMLRPC;
        n.getParam("/" + chainName + "/module_ids", moduleIDs_XMLRPC);
        std::vector<uint8_t> moduleIDs;
        for (int i=0; i<moduleIDs_XMLRPC.size(); i++)
            moduleIDs.push_back(static_cast<int>(moduleIDs_XMLRPC[i]));
	
	//!Put the devices in a chain to array devices
	XmlRpc::XmlRpcValue devices_XMLRPC;
        n.getParam("/" + chainName + "/devices", devices_XMLRPC);
        std::vector<std::string> devices;
        for (int i=0; i<devices_XMLRPC.size(); i++)
            devices.push_back(static_cast<std::string>(devices_XMLRPC[i]));
	
	//what is happening here..?
	//Is it assignment of canopen devices with a specific module ID with its moduleID, jointname , chainname and device name.?
	for (unsigned int i=0; i<jointNames.size(); i++)
            canopen::devices[ moduleIDs[i] ] = canopen::Device(moduleIDs[i], jointNames[i], chainName, devices[i]);

        canopen::deviceGroups[ chainName ] = canopen::DeviceGroup(moduleIDs, jointNames);

    }

}

//!Get maximum velocities,upper limit, lower limit and offsets from the URDF file

/*! The function takes a ROS node as its parameter and returns nothing
\param n is a ROS Node */
void setJointConstraints(ros::NodeHandle n)
{
    //! Get robot_description from ROS parameter server
      joint_limits_ = new JointLimits();
      int DOF = jointNames.size();
	
      //!Declare 3 string variables
      std::string param_name = "/robot_description";
      std::string full_param_name;
      std::string xml_string;
      
      //!Search for the robot_description in the node n
      n.searchParam(param_name, full_param_name);
      if (n.hasParam(full_param_name))
      {
      	//!If found, get the param_name from xml_string and store it in full_param_name. Is this correct..? 
          n.getParam(full_param_name.c_str(), xml_string);
      }

      else
      {   //!If not found, shut down the node
          ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
          n.shutdown();
      }
      
      //!Check for the size of the xml_string
      /*!If the string is null, display an error message and shutdown the ROS Node */
      if (xml_string.size() == 0)
      {
          ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
          n.shutdown();
      }
      
      //!Display the robot_description in case of no error
      ROS_INFO("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

      //!Get urdf model out of robot_description
      urdf::Model model;
      
      //!Parse the URDF file
      
      /*!Check whether the model is initialised with xml_string.
      If not, display an error message and shutdown the ROS Node.
      Otherwise display the message that the URDF file is successfully parsed */
      if (!model.initString(xml_string))
      {
          ROS_ERROR("Failed to parse urdf file");
          n.shutdown();
      }
      ROS_INFO("Successfully parsed urdf file");

      //!Get maximum velocities out of urdf model
      std::vector<double> MaxVelocities(DOF);
      for (int i = 0; i < DOF; i++)
      {
          MaxVelocities[i] = model.getJoint(jointNames[i].c_str())->limits->velocity;
      }

      //! Get lower limits out of urdf model
      std::vector<double> LowerLimits(DOF);
      for (int i = 0; i < DOF; i++)
      {
          LowerLimits[i] = model.getJoint(jointNames[i].c_str())->limits->lower;
      }

      //!Get upper limits out of urdf model
      std::vector<double> UpperLimits(DOF);
      for (int i = 0; i < DOF; i++)
      {
          UpperLimits[i] = model.getJoint(jointNames[i].c_str())->limits->upper;
      }

      //! Get offsets out of urdf model
      std::vector<double> Offsets(DOF);
      for (int i = 0; i < DOF; i++)
      {
          Offsets[i] = model.getJoint(jointNames[i].c_str())->calibration->rising.get()[0];
      }

      //! Set parameters

      joint_limits_->setDOF(DOF);
      joint_limits_->setUpperLimits(UpperLimits);
      joint_limits_->setLowerLimits(LowerLimits);
      joint_limits_->setMaxVelocities(MaxVelocities);
      joint_limits_->setOffsets(Offsets);

    
}


int main(int argc, char **argv)
{
    //! todo: allow identical module IDs of modules when they are on different CAN buses

    //!Calling the ros_init() function to perform ROS arguments and name remapping provided at command line and canopen_ros is the name of the node
    ros::init(argc, argv, "canopen_ros");
    
    //!NodeHAndle is the main point of communication with the ROS system
    ros::NodeHandle n(""); // ("~");
    
    //!Read parameters from the Parameter Server
    
    readParamsFromParameterServer(n);
    
    
    std::cout << "Sync Interval" << buses.begin()->second.syncInterval << std::endl;
    canopen::syncInterval = std::chrono::milliseconds( buses.begin()->second.syncInterval );
    // ^ todo: this only works with a single CAN bus; add support for more buses!
    
    //!Fetching the baudrate of the device..?
    
    deviceFile = buses.begin()->first;
    std::cout << "Opening device..." << deviceFile << std::endl;
    // ^ todo: this only works with a single CAN bus; add support for more buses!
    
    //!Opening the CAN device
    
    if (!canopen::openConnection(deviceFile))
    {
        ROS_ERROR("Cannot open CAN device; aborting.");
        exit(EXIT_FAILURE);
    }
    else
    {
        std::cout << "Connection to CAN bus established" << std::endl;
    }

    canopen::pre_init();

    /********************************************/

    //! add custom PDOs:
    
    canopen::sendPos = canopen::defaultPDOOutgoing;
    for (auto it : canopen::devices) {
        canopen::incomingPDOHandlers[ 0x180 + it.first ] = [it](const TPCANRdMsg m) { canopen::defaultPDO_incoming( it.first, m ); };
        canopen::incomingEMCYHandlers[ 0x081 + it.first ] = [it](const TPCANRdMsg mE) { canopen::defaultEMCY_incoming( it.first, mE ); };
    }

    //! set up services, subscribers, and publishers for each of the chains
   
    std::vector<TriggerType> initCallbacks;
    std::vector<ros::ServiceServer> initServices;
    std::vector<TriggerType> recoverCallbacks;
    std::vector<ros::ServiceServer> recoverServices;
    std::vector<TriggerType> stopCallbacks;
    std::vector<ros::ServiceServer> stopServices;
    std::vector<SetOperationModeCallbackType> setOperationModeCallbacks;
    std::vector<ros::ServiceServer> setOperationModeServices;

    std::vector<JointVelocitiesType> jointVelocitiesCallbacks;
    std::vector<ros::Subscriber> jointVelocitiesSubscribers;
    std::map<std::string, ros::Publisher> currentOperationModePublishers;
    std::map<std::string, ros::Publisher> statePublishers;
    ros::Publisher jointStatesPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Publisher diagnosticsPublisher = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    for (auto it : canopen::deviceGroups)
    {
    	//!How does it refer to the chainname....?
    	
        ROS_INFO("Configuring %s", it.first.c_str());
	
	//!A number of times when CANopenInit() is called. Why is it it.first.?
	
        initCallbacks.push_back( boost::bind(CANopenInit, _1, _2, it.first) );
        initServices.push_back( n.advertiseService("/" + it.first + "/init", initCallbacks.back()) );
        recoverCallbacks.push_back( boost::bind(CANopenRecover, _1, _2, it.first) );
        recoverServices.push_back( n.advertiseService("/" + it.first + "/recover", recoverCallbacks.back()) );
        stopCallbacks.push_back( boost::bind(CANOpenHalt, _1, _2, it.first) );
        stopServices.push_back( n.advertiseService("/" + it.first + "/halt", stopCallbacks.back()) );
        
        //!setOperationModeCallback is a dummy function. Not yet implemented
        
        setOperationModeCallbacks.push_back( boost::bind(setOperationModeCallback, _1, _2, it.first) );
        setOperationModeServices.push_back( n.advertiseService("/" + it.first + "/set_operation_mode", setOperationModeCallbacks.back()) );

        jointVelocitiesCallbacks.push_back( boost::bind(setVel, _1, it.first) );
        jointVelocitiesSubscribers.push_back( n.subscribe<brics_actuator::JointVelocities>("/" + it.first + "/command_vel", 1, jointVelocitiesCallbacks.back()) );

        currentOperationModePublishers[it.first] = n.advertise<std_msgs::String>("/" + it.first + "/current_operationmode", 1);

        statePublishers[it.first] = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/" + it.first + "/state", 1);
    }

    double lr = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(canopen::syncInterval).count();
    
    //!Loops at 1000 Hz
    ros::Rate loop_rate(lr);

    setJointConstraints(n);

    while (ros::ok())
    {

    //!iterate over all chains, get current pos and vel and publish as topics:
	int counter = 0;
        std::vector <double> positions;
        std::vector <double> desired_positions;

        for (auto device : canopen::devices)
        {
		
            double pos = (double)device.second.getActualPos() + joint_limits_->getOffsets()[counter];
            double des_pos = (double)device.second.getDesiredPos() + joint_limits_->getOffsets()[counter];
            positions.push_back(pos);
            desired_positions.push_back(des_pos);
	    counter++;
        }
	
	//!Iterate over all chains
        for (auto dg : (canopen::deviceGroups))
        {
        	
            //!Getting all the 4 attributes like name, position, velocit and effort
            
            sensor_msgs::JointState js;
            js.name = dg.second.getNames();
            
            //!What is this..?
            
            js.header.stamp = ros::Time::now(); // todo: possibly better use timestamp of hardware msg?
	    
            js.position = positions;//dg.second.getActualPos();
            //!std::cout << "Position" << js.position[0] << std::endl;
            js.velocity = dg.second.getActualVel();
            js.effort = std::vector<double>(dg.second.getNames().size(), 0.0);
            
            //!Publish the attributes as topics
            
            jointStatesPublisher.publish(js);

            pr2_controllers_msgs::JointTrajectoryControllerState jtcs;
            jtcs.header.stamp = js.header.stamp;
            jtcs.actual.positions = js.position;
            jtcs.actual.velocities = js.velocity;
            jtcs.desired.positions = desired_positions;//dg.second.getDesiredPos();
            jtcs.desired.velocities = dg.second.getDesiredVel();
            
            //!What does this mean?
            statePublishers[dg.first].publish(jtcs);

            std_msgs::String opmode;
            opmode.data = "velocity";
            currentOperationModePublishers[dg.first].publish(opmode);
	    counter++;
        }

        //!publishing diagnostic messages
        diagnostic_msgs::DiagnosticArray diagnostics;
        diagnostic_msgs::DiagnosticStatus diagstatus;
        std::vector<diagnostic_msgs::DiagnosticStatus> diagstatus_msg;
        diagnostic_msgs::KeyValue keyval;

        std::vector<diagnostic_msgs::KeyValue> keyvalues;


	//!What is the function of resize() and what is the use of this statement.?
        diagnostics.status.resize(1);

    for (auto dg : (canopen::devices))
    {
    	//!dg.second.getNAme()..?
    	
        std::string name = dg.second.getName();
        //!ROS_INFO("Name %s", name.c_str() );

        keyval.key = "Node ID";
        uint16_t node_id = dg.second.getCANid();
        std::stringstream result;
        result << node_id;
        keyval.value = result.str().c_str();
        keyvalues.push_back(keyval);
	
	//!Get the HW Version and store it in the array keyvalues
        keyval.key = "Hardware Version";
        
        //!Where is this defined..? Is it in Canopen_core..?
        std::vector<char> manhw = dg.second.getManufacturerHWVersion();
        keyval.value = std::string(manhw.begin(), manhw.end());
        keyvalues.push_back(keyval);

	//!Get the Manufacturer SW Version and store it in the array keyvalues
        keyval.key = "Software Version";
        std::vector<char> mansw = dg.second.getManufacturerSWVersion();
        keyval.value = std::string(mansw.begin(), mansw.end());
        keyvalues.push_back(keyval);

	//!Get the Device Name and store it in the array keyvalues
        keyval.key = "Device Name";
        std::vector<char> dev_name = dg.second.getManufacturerDevName();
        keyval.value = std::string(dev_name.begin(), dev_name.end());
        keyvalues.push_back(keyval);

	//!Get the Vendor ID and store it in the array keyvalues
        keyval.key = "Vendor ID";
        std::vector<uint16_t> vendor_id = dg.second.getVendorID();
        std::stringstream result1;
        for (auto it : vendor_id)
        {
           result1 <<  std::hex << it;
        }
        keyval.value = result1.str().c_str();
        keyvalues.push_back(keyval);

	//!Get the Revision Number and store it in the array keyvalues
        keyval.key = "Revision Number";
        uint16_t rev_number = dg.second.getRevNumber();
        std::stringstream result2;
        result2 << rev_number;
        keyval.value = result2.str().c_str();
        keyvalues.push_back(keyval);

	//!Get the Product Code and store it in the array keyvalues
        keyval.key = "Product Code";
        std::vector<uint16_t> prod_code = dg.second.getProdCode();
        std::stringstream result3;
        std::copy(prod_code.begin(), prod_code.end(), std::ostream_iterator<uint16_t>(result3, " "));
        keyval.value = result3.str().c_str();
        keyvalues.push_back(keyval);


	//!Where is this function defined.?
        bool error_ = dg.second.getFault();
        bool initialized_ = dg.second.getInitialized();

        //ROS_INFO("Fault: %d", error_);
        //ROS_INFO("Referenced: %d", initialized_);

        //! set data to diagnostics
        
        //!Set error_ to 1 if fault is detected and the corresponding diagnostic status is also set
        if(error_)
        {
          diagstatus.level = 2;
          diagstatus.name = chainNames[0];
          diagstatus.message = "Fault occured.";
          diagstatus.values = keyvalues;
          break;
        }
        else
        {
          //!set the diagnostic status to OK if the device is initialised, and no error is reported, and also update all the details
          if (initialized_)
          {
            diagstatus.level = 0;
            diagstatus.name = chainNames[0];
            diagstatus.message = "powerball chain initialized and running";
            diagstatus.values = keyvalues;
          }
          else
          {
            diagstatus.level = 1;
            diagstatus.name = chainNames[0];
            diagstatus.message = "powerball chain not initialized";
            diagstatus.values = keyvalues;
            break;
          }
        }
    }
        diagstatus_msg.push_back(diagstatus);
        
        //!publish diagnostic message
        diagnostics.status = diagstatus_msg;
        diagnostics.header.stamp = ros::Time::now();
        diagnosticsPublisher.publish(diagnostics);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

