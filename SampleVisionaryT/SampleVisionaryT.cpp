//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: November 2016
// 
// @author:  Andreas Richert
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de
// 
// Last commit: $Date: 2018-02-15 13:51:24 +0100 (Do, 15 Feb 2018) $
// Last editor: $Author: richean $
// 
// Version "$Revision: 16012 $"
//

#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "VisionaryTData.h"    // Header specific for the Time of Flight data
#include "VisionaryDataStream.h"
#include "VisionaryControl.h"
#include "PointXYZ.h"
#include "PointCloudPlyWriter.h"
#include "CoLaBCommandBuilder.h"
#include "CoLaBCommandReader.h"
#include <vector>

//void calculatevolume(std::vector<PointXYZ> pointCloud)
//{
	////std::cout << 'rows: ' << distanceMap.size << '\n\n';
	//printf("hi! \n\n");
	////printf("columns: %d \n\n", pointCloud[100]);
	//for(int i=0; i < pointCloud.size(); i++) printf("%d \n", pointCloud[i].z);
	
	
	////for (std::vector<uint16_t>::const_iterator i = distanceMap.begin(); i != distanceMap.end(); ++i) std::cout << *i << ' ';
}



bool runStreamingDemo(char* ipAddress, unsigned short port)
{
  // Generate Visionary instance
  boost::shared_ptr<VisionaryTData> pDataHandler = boost::make_shared<VisionaryTData>();
  VisionaryDataStream dataStream(pDataHandler, inet_addr(ipAddress), htons(port));
  VisionaryControl control(inet_addr(ipAddress), htons(2112));
  
  //-----------------------------------------------
  // Connect to devices data stream 
  if (!dataStream.openConnection())
  {
    printf("Failed to open data stream connection to device.\n");
    return false;   // connection failed
  }

  //-----------------------------------------------
  // Connect to devices control channel
  if (!control.openConnection())
  {
    printf("Failed to open control connection to device.\n");
    return false;   // connection failed
  }

  //-----------------------------------------------
  // Login as authorized client
  if (control.login(CoLaUserLevel::AUTHORIZED_CLIENT, "CLIENT"))
  {
    //-----------------------------------------------
    // An example of reading an writing device parameters is shown here.
    // Use the "SOPAS Communication Interface Description" PDF to determine data types for other variables

    //-----------------------------------------------
    // Set integrationTimeUs parameter to 150000
    printf("Setting integrationTimeUs to 3800\n");
    CoLaBCommand setIngrationTimeCommand = CoLaBCommandBuilder(CoLaCommandType::WRITE_VARIABLE, "integrationTimeUs").parameterUDInt(3800).build();
    CoLaBCommand setIngrationTimeResponse = control.sendCommand(setIngrationTimeCommand);

    //-----------------------------------------------
    // Read integrationTimeUs parameter
    CoLaBCommand getIntegrationTimeCommand = CoLaBCommandBuilder(CoLaCommandType::READ_VARIABLE, "integrationTimeUs").build();
    CoLaBCommand integrationTimeResponse = control.sendCommand(getIntegrationTimeCommand);
    uint32_t integrationTimeUs = CoLaBCommandReader(integrationTimeResponse).readUDInt();
    printf("Read integrationTimeUs = %d\n", integrationTimeUs);

    //-----------------------------------------------
    // Read info messages variable
    CoLaBCommand getMessagesCommand = CoLaBCommandBuilder(CoLaCommandType::READ_VARIABLE, "MSinfo").build();
    CoLaBCommand messagesResponse = control.sendCommand(getMessagesCommand);

    //-----------------------------------------------
    // Read message array, length of array is always 25 items (see PDF).
    CoLaBCommandReader reader(messagesResponse);
    for (int i = 0; i < 25; i++)
    {
      uint32_t errorId = reader.readUDInt();
      uint32_t errorState = reader.readUDInt();
      uint16_t firstTimePwrOnCount = reader.readUInt();
      uint32_t firstTimeOpSecs = reader.readUDInt();
      uint32_t firstTimeTimeOccur = reader.readUDInt();
      uint16_t lastTimePwrOnCount = reader.readUInt();
      uint32_t lastTimeOpSecs = reader.readUDInt();
      uint32_t lastTimeTimeOccur = reader.readUDInt();
      uint16_t numberOccurance = reader.readUInt();
      uint16_t errReserved = reader.readUInt();
      std::string extInfo = reader.readFlexString();

      if (errorId != 0)
      {
        printf("Info message [0x%032x], extInfo: %s, numberOccurance: %d\n", errorId, extInfo.c_str(), numberOccurance);
      }
    }
  }

  //-----------------------------------------------
  // Logout from device after reading variables.
  if (!control.logout())
  {
    printf("Failed to logout\n");
  }

  //-----------------------------------------------
  // Stop image acquisition (works always, also when already stopped)
  control.stopAcquisition();

  //-----------------------------------------------
  // Capture a single image
  control.stepAcquisition();
  if (dataStream.getNextFrame())
  {
    printf("Frame received through step called, frame #%d, timestamp: %I64u \n", pDataHandler->getFrameNum(), pDataHandler->getTimestampMS());

    //-----------------------------------------------
    // Convert data to a point cloud
    std::vector<PointXYZ> pointCloud;
    pDataHandler->generatePointCloud(pointCloud);
    
	//calculatevolume(pointCloud);

	char* plyFilePath = "/home/rodrigo/Volumetry/volumetry-visionaryt/testcloud.ply";
    printf("Writing frame to %s\n", plyFilePath);
    PointCloudPlyWriter::WriteFormatPLY(plyFilePath, pointCloud, true);
    printf("Finished writing frame to %s\n", plyFilePath);


  control.stopAcquisition();
  control.closeConnection();
  dataStream.closeConnection();
  return true;
  }
}


int main()
{
  // Insert IP of your camera and the API port
  /// Default values:
  /// IP:        "192.168.1.10"
  /// API-port:  2114
  runStreamingDemo("192.168.15.40", 2114);
}
