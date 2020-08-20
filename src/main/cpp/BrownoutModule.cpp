#include "BrownoutModule.h"

void BrownoutModule::periodicInit(){

    pdp = new frc::PowerDistributionPanel{0};

    this->msInterval = BrownoutModuleRunInterval;
    this->ErrorModulePipe = pipes[0];
	this->DriveBaseModulePipe = pipes[1];
    energyThisMatch = 0;

    energyInStream.open(energyLog);
    double tmpTime, tmpEnergy = 0;
     
    if(energyInStream.peek() == std::ifstream::traits_type::eof()){ //if is empty
        fileEmpty = true;
    }
    else {
        energyInStream >> tmpTime;
        energyInStream >> tmpEnergy;

        energyLogTime.push_back(tmpTime);
        pastEnergyLog.push_back(tmpEnergy);
        
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

    double startTime = frc::Timer().GetFPGATimestamp();
    accumulatePower(frc::Timer().GetFPGATimestamp());

    if(isBrownout()){
        ErrorModulePipe->pushQueue(new Message("Brownout will occur!", FATAL));
        DriveBaseModulePipe->pushQueue(new Message("", getDriveCurrentLimitScaling()));
    }

    if(!(writeData(fileName))){
        ErrorModulePipe->pushQueue(new Message("Failed to write Brownout data to file", LOW));
    }

}

/* writes data to csv file */
bool BrownoutModule::writeData(std::string fileName){
    
    if (myFile <<  frc::Timer().GetFPGATimestamp() << ", " << pdp->GetTotalCurrent() << ", "<< pdp->GetVoltage() << ", " << getBatteryPower() << ", " << energyThisMatch << std::endl){
        
        myFile.close();
        return true;
    }
    return false;

}

/* gets battery power */
double BrownoutModule::getBatteryPower(){
    return frc::DriverStation::GetInstance().GetBatteryVoltage() * frc::DriverStation::GetInstance().GetBatteryVoltage() / getBatteryResistance(); 
}

bool BrownoutModule::willBrownOut(){
    
    return frc::DriverStation::GetInstance().GetBatteryVoltage() - batteryResistance * (totalCurrLimit - getMotorCurrentDraw()); 

}

double BrownoutModule::getBatteryResistance(){

    return batteryResistance;
}

/* applies load to drivetrain then measures voltage and current */
void BrownoutModule::calculateBatteryResistance(){

    std::vector<double> currentData;
    std::vector<double> voltageData;

    double currSetpoint = 5;

    DriveBaseModulePipe->pushQueue(new Message("", currSetpoint));

    double startTime = frc::Timer().GetFPGATimestamp();
    double TIME_OUT = 1000; 
    int n = 0;

    if(DriveBaseModulePipe->popQueue()->val == HIGH){ 

        ErrorModulePipe->pushQueue(new Message("Failed to set motors for Brownout Module", HIGH));
    }
    else {
         
        //collect voltage and current
        while (frc::Timer().GetFPGATimestamp() - startTime < TIME_OUT){
            
            getLineOfBestFitSlope(pdp->GetTotalCurrent(), pdp->GetVoltage());
            DriveBaseModulePipe->pushQueue(new Message("", currSetpoint));
            currSetpoint++;
            n++;
        }

    }

    batteryResistance = (n*xysum - xsum*ysum)/(n*x2sum - xsum*xsum);

}

/* uses least squares to find the slope of the line of best fit */
void BrownoutModule::getLineOfBestFitSlope(double curr, double volt){

    xsum += curr;                        //calculate sigma(xi)
    ysum += volt;                       //calculate sigma(yi)
    x2sum += pow(curr,2);                //calculate sigma(x^2i)
    xysum += curr * volt;                    //calculate sigma(xi*yi)

   //double slope = (n*xysum - xsum*ysum)/(n*x2sum - xsum*xsum);            //calculate slope
  //return slope;
}

bool BrownoutModule::isBrownout(){
    
    return frc::RobotController::IsBrownedOut();

}

bool BrownoutModule::accumulatePower(double time){

    if(fileEmpty) return false;

    energyStream << time << ", "<< pdp->GetTotalEnergy() << std::endl;

    for(int i = 1; i < energyLogTime.size(); i++){
        if(energyLogTime.at(i - 1) <= time && time <= energyLogTime.at(i)){
            return (pdp->GetTotalEnergy() > pastEnergyLog.at(i));
        }
    }

}

//add motors
double BrownoutModule::getMotorCurrentDraw(){
    return pdp->GetCurrent(LMOTOR_LEAD_CHANNEL) + pdp->GetCurrent(RMOTOR_LEAD_CHANNEL) + 
        pdp->GetCurrent(LMOTOR_FOLLOWER_CHANNEL) + pdp->GetCurrent(RMOTOR_FOLLOWER_CHANNEL);
}

double BrownoutModule::getDriveCurrentLimitScaling(){

    //scaling factor to avoid brownout
    double scaling =  (VOLTAGE_THRESHOLD - pdp->GetVoltage())/(getBatteryResistance() * (totalCurrLimit - getMotorCurrentDraw())); 
    scaling *= (100 - nonMotorCurrent)/100.0; //scale drive

    if (scaling > 1)
        return 1;

    return scaling;

}

std::vector<uint8_t> BrownoutModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID, DriveBaseModuleID, UpdateMotorModuleID}; }