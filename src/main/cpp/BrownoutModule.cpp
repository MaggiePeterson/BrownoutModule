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

    if(stateRef->IsTest() && !hasRun){
        calculateBatteryResistance();
        hasRun = true;
    }

    double startTime = frc::Timer().GetFPGATimestamp();
    accumulatePower(frc::Timer().GetFPGATimestamp() - startTime);

    if(isBrownout()){
        ErrorModulePipe->pushQueue(new Message("Brownout will occur!", FATAL));
        DriveBaseModulePipe->pushQueue(new Message("", getCurrentLimitScaling()));
    }

    if(!(writeData(fileName))){
        ErrorModulePipe->pushQueue(new Message("Failed to write Brownout data to file", LOW));
    }

}

/* writes data to csv file */
bool BrownoutModule::writeData(std::string fileName){

    std::ofstream myFile;
    myFile.open (fileName);
    
    if (myFile <<  frc::Timer().GetFPGATimestamp() << ", " << pdp->GetTotalCurrent() << ", "<< pdp->GetVoltage() << ", " << getBatteryPower() << ", " << energyThisMatch << std::endl){
        
        myFile.close();
        return true;
    }
    myFile.close();
    return false;

}

/* gets battery power */
double BrownoutModule::getBatteryPower(){
    return frc::DriverStation::GetInstance().GetBatteryVoltage() * frc::DriverStation::GetInstance().GetBatteryVoltage() / getBatteryResistance(); 
}

double BrownoutModule::getMaxCurrentDraw(){
    
    return pdp->GetVoltage() - getBatteryResistance() * getMotorCurrentDraw();; 

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

    double startTime; 
    double TIME_OUT; 

    if(DriveBaseModulePipe->popQueue()->val == HIGH){ 

        ErrorModulePipe->pushQueue(new Message("Failed to set motors for Brownout Module", HIGH));
    }
    else {
         
        startTime = frc::Timer().GetFPGATimestamp();
        TIME_OUT = 1000;

        //collect voltage and current
        while (frc::Timer().GetFPGATimestamp() - startTime < TIME_OUT){
            
            currentData.push_back(pdp->GetTotalCurrent());
            voltageData.push_back(pdp->GetVoltage());
            DriveBaseModulePipe->pushQueue(new Message("", currSetpoint));
            currSetpoint++;

        }

    }

    batteryResistance = getLineOfBestFitSlope(currentData, voltageData);

}

/* uses least squares to find the slope of the line of best fit */
double BrownoutModule::getLineOfBestFitSlope(std::vector<double> xData, std::vector<double> yData){

    if(xData.size() != yData.size())
        ErrorModulePipe->pushQueue(new Message("Could not calculate battery resistance!", FATAL));

   double n = xData.size();
   double xsum = 0.0, x2sum = 0.0, ysum = 0.0, xysum = 0.0; //variables for sums/sigma of xi,yi,xi^2,xiyi etc

   for (int i = 0; i < n; i++)
   {
      xsum += xData[i];                        //calculate sigma(xi)
      ysum += yData[i];                       //calculate sigma(yi)
      x2sum += pow(xData[i],2);                //calculate sigma(x^2i)
      xysum += xData[i] * xData[i];                    //calculate sigma(xi*yi)
   }

   double slope = (n*xysum - xsum*ysum)/(n*x2sum - xsum*xsum);            //calculate slope
   return slope;
}

bool BrownoutModule::isBrownout(){
    
    return frc::RobotController::IsBrownedOut();

}

void BrownoutModule::accumulatePower(double deltaTime){

    energyThisMatch += getBatteryPower()*deltaTime;

}

double BrownoutModule::getMotorCurrentDraw(){
    return pdp->GetCurrent(LMOTOR_LEAD_CHANNEL) + pdp->GetCurrent(RMOTOR_LEAD_CHANNEL) + 
        pdp->GetCurrent(LMOTOR_FOLLOWER_CHANNEL) + pdp->GetCurrent(RMOTOR_FOLLOWER_CHANNEL);
}

double BrownoutModule::getCurrentLimitScaling(){

    //scaling factor to avoid brownout
    double scaling =  (VOLTAGE_THRESHOLD - pdp->GetVoltage())/(getBatteryResistance() * getMotorCurrentDraw());
    
    if (scaling >= 1 || scaling < 0)
        return 1;

    return scaling;

}


std::vector<uint8_t> BrownoutModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID, DriveBaseModuleID}; }