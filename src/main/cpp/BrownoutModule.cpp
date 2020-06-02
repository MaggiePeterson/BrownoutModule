#include "BrownoutModule.h"

void BrownoutModule::periodicInit(){}
void BrownoutModule::periodicRoutine(){}

std::vector<uint8_t> BrownoutModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID}; }
