//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: August 2017
// 
// @author:  Andreas Richert
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de
// 
// Last commit: $Date: 2017-12-14 16:56:05 +0100 (Do, 14 Dez 2017) $
// Last editor: $Author: richean $
// 
// Version "$Revision: 15262 $"
//


#include "VisionaryAutoIPScan.h"



int main()
{
  unsigned int timeout = 5000;
  VisionaryAutoIPScan ipScan;

  std::vector<VisionaryAutoIPScan::DeviceInfo> deviceList = ipScan.doScan(timeout);
  for(auto it : deviceList)
  {
    printf("DT: %s \n", it.DeviceName.c_str());
    printf("MAC Address: %s \n", it.MacAddress.c_str());
    printf("IP Address: %s \n", it.IpAddress.c_str());
    printf("Subnet: %s \n", it.SubNet.c_str());
    printf("Port %s \n", it.Port.c_str());
  }
  return 0;
}
