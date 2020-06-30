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

    if(isBrownout()){
        ErrorModulePipe->pushQueue(new Message("Brownout will occur!", FATAL));
    }

    writeData(fileName);
}

/* writes data to csv file */
void BrownoutModule::writeData(std::string fileName){

    std::ofstream myFile;
    myFile.open (fileName);
    if (myFile <<  frc::Timer().GetFPGATimestamp() << pdp->GetTotalCurrent() << pdp->GetVoltage() << getBatteryPower() << std::endl){
        myFile.close();
        //return true;
    }
    

}

/* gets battery power */
double BrownoutModule::getBatteryPower(){
    return frc::DriverStation::GetInstance().GetBatteryVoltage() * frc::DriverStation::GetInstance().GetBatteryVoltage() / getBatteryResistance(); 
}

double BrownoutModule::getMaxCurrentDraw(){
    
    return 0; 

}

double BrownoutModule::getBatteryResistance(){

    return batteryResistance;
}

/* applies load to drivetrain then measures voltage and current */
void BrownoutModule::calculateBatteryResistance(){

    std::vector<double> currentData;
    std::vector<double> voltageData;

    double moveSetpoint = 10;

    DriveBaseModulePipe->pushQueue(new Message("", moveSetpoint));

    double startTime = frc::Timer().GetFPGATimestamp();
    double TIME_OUT = 1000;

    while (frc::Timer().GetFPGATimestamp() - startTime < TIME_OUT){
        currentData.push_back(pdp->GetTotalCurrent());
        voltageData.push_back(pdp->GetVoltage());
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

/* checks if current draw will drop voltage below 7V */
bool BrownoutModule::isBrownout(){
    
    double potentialVoltDrop = frc::DriverStation::GetInstance().GetBatteryVoltage() - getBatteryResistance()*pdp->GetTotalCurrent();
    return potentialVoltDrop < VOLTAGE_THRESHOLD;
}

/* calc watt hours

voltage provides * current can provide for time

*/ 

std::vector<uint8_t> BrownoutModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID, DriveBaseModuleID}; }