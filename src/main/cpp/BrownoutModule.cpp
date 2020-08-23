#include "BrownoutModule.h"

void BrownoutModule::periodicInit(){

    pdp = new frc::PowerDistributionPanel{0};

    this->msInterval = BrownoutModuleRunInterval;
    this->ErrorModulePipe = pipes[0];
	this->DriveBaseModulePipe = pipes[1];

    energyInStream.open(energyLog);
    double tmpTime, tmpEnergy = 0;
     
    if(energyInStream.peek() == std::ifstream::traits_type::eof()){ //if is empty
        fileEmpty = true;
    }
    else {

        while(energyInStream >> tmpTime){
        
            energyInStream >> tmpEnergy;

            timestamp.push_back(tmpTime);
            pastEnergy.push_back(tmpEnergy);
        }
    }

    energyInStream.close();

    myFile.open (fileName);
    energyStream.open (energyLog);

}

void BrownoutModule::periodicRoutine(){

     if (!errors.empty()) { // Handle internal ModuleBase Errors
        ErrorModulePipe->pushQueue(errors.front());
        errors.pop();
    }

    if(stateRef->IsDisabled()) return;

    if(stateRef->IsTest() && !hasRun){
        calculateBatteryResistance();
        hasRun = true;
    }

    //Errors saved to file by ErrorModule w/ timestamp
    if(checkEnergy(frc::Timer().GetFPGATimestamp()) || willBrownOut()){
        DriveBaseModulePipe->pushQueue(new Message("CURRENT", getDriveCurrentLimitScaling()));
        ErrorModulePipe->pushQueue(new Message(std::to_string(frc::Timer().GetFPGATimestamp()) + ": Lowering Current Limit  - Exceeding Energy", INFO));
    }
    if(willBrownOut()){
        DriveBaseModulePipe->pushQueue(new Message("CURRENT", getDriveCurrentLimitScaling()));
        ErrorModulePipe->pushQueue(new Message(std::to_string(frc::Timer().GetFPGATimestamp()) + ": Lowering Current Limit - Voltage Low", INFO));
    }
    if(isBrownout()){
        ErrorModulePipe->pushQueue(new Message(std::to_string(frc::Timer().GetFPGATimestamp()) + ": Brownout occuring!", FATAL));
        DriveBaseModulePipe->pushQueue(new Message("CURRENT", getDriveCurrentLimitScaling()));
    }

    if(!(writeData(fileName))){
        ErrorModulePipe->pushQueue(new Message("Failed to write Brownout data to file", LOW));
    }

}

/* writes data to csv file */
bool BrownoutModule::writeData(std::string fileName){
    
    if (myFile <<  frc::Timer().GetFPGATimestamp() << ", " << pdp->GetTotalCurrent() << ", "<< pdp->GetVoltage() << ", " << getBatteryPower() << std::endl){
        
        myFile.close();
        return true;
    }
    return false;

}

/* gets battery power */
double BrownoutModule::getBatteryPower(){
    return frc::DriverStation::GetInstance().GetBatteryVoltage() * frc::DriverStation::GetInstance().GetBatteryVoltage() / batteryResistance; 
}

bool BrownoutModule::willBrownOut(){
    
    return frc::DriverStation::GetInstance().GetBatteryVoltage() - batteryResistance * (totalCurrLimit - getMotorCurrentDraw()); 

}

/* applies load to drivetrain then measures voltage and current */
void BrownoutModule::calculateBatteryResistance(){

    std::vector<double> currentData;
    std::vector<double> voltageData;

    double currSetpoint = 5;

    DriveBaseModulePipe->pushQueue(new Message("", currSetpoint));

    double startTime = frc::Timer().GetFPGATimestamp();
    double TIME_OUT = 100; 
    int n = 0;

    if(DriveBaseModulePipe->popQueue()->val == HIGH){ 

        ErrorModulePipe->pushQueue(new Message("Failed to set motors for Brownout Module", HIGH));
    }
    else {
         
        //collect voltage and current
        while (frc::Timer().GetFPGATimestamp() - startTime < TIME_OUT){
            
            takeSum(pdp->GetTotalCurrent(), pdp->GetVoltage());
            DriveBaseModulePipe->pushQueue(new Message("", currSetpoint));
            currSetpoint++;
            n++;
        }

    }

    batteryResistance = (n*xysum - xsum*ysum)/(n*x2sum - xsum*xsum);

}

/* uses least squares to find the slope of the line of best fit */
void BrownoutModule::takeSum(double curr, double volt){

    xsum += curr;                        //calculate sigma(xi)
    ysum += volt;                       //calculate sigma(yi)
    x2sum += pow(curr,2);                //calculate sigma(x^2i)
    xysum += curr * volt;                    //calculate sigma(xi*yi)
}

bool BrownoutModule::isBrownout(){
    
    return frc::RobotController::IsBrownedOut();
}

bool BrownoutModule::checkEnergy(double time){

    if(fileEmpty) return false;

    energyStream << time << ", "<< pdp->GetTotalEnergy() << std::endl;

    //compare around the same time
    for(uint8_t i = 1; i < pastEnergy.size(); i++){
        if(timestamp.at(i - 1) <= time && time <= timestamp.at(i)){
            return (pdp->GetTotalEnergy() > pastEnergy.at(i));
        }
    }

}

//add motor channels
double BrownoutModule::getMotorCurrentDraw(){
    return pdp->GetCurrent(LMOTOR_LEAD_CHANNEL) + pdp->GetCurrent(RMOTOR_LEAD_CHANNEL) + 
        pdp->GetCurrent(LMOTOR_FOLLOWER_CHANNEL) + pdp->GetCurrent(RMOTOR_FOLLOWER_CHANNEL);
}

double BrownoutModule::getDriveCurrentLimitScaling(){

    //scaling factor to avoid brownout
    double scaling =  (VOLTAGE_THRESHOLD - pdp->GetVoltage())/(batteryResistance * (totalCurrLimit - getMotorCurrentDraw())); 
    scaling *= (100 - nonDriveLoad)/100.0; //scale drive

    if (scaling > 1)
        return 1;

    return scaling;

}

std::vector<uint8_t> BrownoutModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID, DriveBaseModuleID, UpdateMotorModuleID}; }