#include "BrownoutModule.h"

void BrownoutModule::periodicInit(){

    pdp = new frc::PowerDistributionPanel{0};

    this->msInterval = BrownoutModuleRunInterval;
    this->ErrorModulePipe = pipes[0];
	this->DriveBaseModulePipe = pipes[1];
}

void BrownoutModule::writeData(std::string fileName){

    std::ofstream myFile;
    myFile.open (fileName);
    myFile <<  frc::Timer().GetFPGATimestamp() << pdp->GetTotalCurrent() << pdp->GetVoltage() << getBatteryPower() << std::endl;
    myFile.close();

}

void BrownoutModule::periodicRoutine(){

     if (!errors.empty()) { // Handle internal ModuleBase Errors
        ErrorModulePipe->pushQueue(errors.front());
        errors.pop();
    }

    writeData(fileName);

}

double BrownoutModule::getBatteryPower(){
    return frc::DriverStation::GetInstance().GetBatteryVoltage() * frc::DriverStation::GetInstance().GetBatteryVoltage() / getBatteryResistance(); 
}

double BrownoutModule::getMaxCurrentDraw(){
    // max current draw with > 7V in battery

    // calc find total resistance and calculate amp at which V is 7,  
    return getBatteryPower()/VOLTAGE_THRESHOLD; 

}

double BrownoutModule::getBatteryResistance(){

    return frc::DriverStation::GetInstance().GetBatteryVoltage()/ pdp->GetTotalCurrent();
}


bool BrownoutModule::isBrownout(){
    
    return frc::DriverStation::GetInstance().GetBatteryVoltage() < 7.0;
}


std::vector<uint8_t> BrownoutModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID, DriveBaseModuleID}; }
