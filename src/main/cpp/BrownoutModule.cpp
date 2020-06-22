#include "BrownoutModule.h"

void BrownoutModule::periodicInit(){

    pdp = new frc::PowerDistributionPanel{0};

    this->msInterval = BrownoutModuleRunInterval;
    this->ErrorModulePipe = pipes[0];
	this->DriveBaseModulePipe = pipes[1];
}

void BrownoutModule::periodicRoutine(){

     if (!errors.empty()) { // Handle internal ModuleBase Errors
        ErrorModulePipe->pushQueue(errors.front());
        errors.pop();
    }

    if(stateRef->IsDisabled()) return;

    if(isBrownout()){
        ErrorModulePipe->pushQueue(new Message("Brownout will occur!", FATAL));
    }

    writeData(fileName);

}

void BrownoutModule::writeData(std::string fileName){

    std::ofstream myFile;
    myFile.open (fileName);
    myFile <<  frc::Timer().GetFPGATimestamp() << pdp->GetTotalCurrent() << pdp->GetVoltage() << getBatteryPower() << std::endl;
    myFile.close();

}

double BrownoutModule::getBatteryPower(){
    return frc::DriverStation::GetInstance().GetBatteryVoltage() * frc::DriverStation::GetInstance().GetBatteryVoltage() / getBatteryResistance(); 
}

double BrownoutModule::getMaxCurrentDraw(){
    // max current draw with > 7V in battery

    /* you'd need to find the net resistance of the total system i think
 then calculate the amperage at which the voltage is at 7V
 subtract that from the batteries capacity
 and then given a specific time limit you can calculate the rate of discharge

*/
    return getBatteryPower()/VOLTAGE_THRESHOLD; 

}

double BrownoutModule::getBatteryResistance(){

    //does I need to be current at the battery or total current?
    //voltage should be voltage drops
    return frc::DriverStation::GetInstance().GetBatteryVoltage()/ pdp->GetTotalCurrent();
}


bool BrownoutModule::isBrownout(){
    
    double potentialVoltDrop = frc::DriverStation::GetInstance().GetBatteryVoltage() - getBatteryResistance()*pdp->GetTotalCurrent();
    return potentialVoltDrop < VOLTAGE_THRESHOLD;
}

std::vector<uint8_t> BrownoutModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID, DriveBaseModuleID}; }
