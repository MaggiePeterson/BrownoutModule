#include "UpdateMotorModule.h"



void UpdateMotorModule::periodicInit() {
  
}

void UpdateMotorModule::periodicRoutine() {
  
}

std::vector<uint8_t> UpdateMotorModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID, BrownoutModuleID}; }
