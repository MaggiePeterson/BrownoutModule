#include "BrownoutModule.h"

void BrownoutModule::periodicInit(){

    pdp = new frc::PowerDistributionPanel{0};

    this->msInterval = BrownoutModuleRunInterval;
    this->ErrorModulePipe = pipes[0];
	this->DriveBaseModulePipe = pipes[1];

    lastMatchStream.open(lastMatchFile);
    double tmpTime, tmpEnergy = 0;
     
    if(lastMatchStream.peek() == std::ifstream::traits_type::eof()){ //if is empty
        fileEmpty = true;
    }
    else {
        compilePastMatchData();
    }

    generalStream.open (generalFile);
    lastMatchStream.open(lastMatchFile);
    allMatchesStream.open (allMatchesFile);

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
    //use match timer
    if(checkEnergy(frc::Timer().GetFPGATimestamp(), frc::Timer().GetMatchTime()) || willBrownOut()){
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

    if(!(writeData(generalFile))){
        ErrorModulePipe->pushQueue(new Message("Failed to write Brownout data to file", LOW));
    }

    //close files at end
    if(frc::Timer().GetMatchTime() == 0){
        allMatchesStream.close();
        generalStream.close();
        lastMatchStream.close();
    }

}

void BrownoutModule::compilePastMatchData(){

    //read in raw match data from the past match
    double time, matchTime, energyRecord;
    std::vector<double> timeStamp;
    std::vector<double> energyInMatch;

    while(lastMatchStream >> time){

        lastMatchStream >> matchTime;
        lastMatchStream >> energyRecord;

        timeStamp.push_back(matchTime);
        energyInMatch.push_back(energyRecord);

    }

    //clear
    lastMatchStream.open(lastMatchFile, std::ofstream::out | std::ofstream::trunc);
 
    std::vector<double> avgEnergyInterval;
    //If match was completed (countdown reached 0)
    if(timeStamp.back() == 0 ) {

        //Average the energy for each time interval
        double avgRange = 0, upperBound = BrownoutModuleRunInterval/1000.0, lowerBound = 0;
        
        uint8_t i = 0;
        int n = 0;

        while(i < timeStamp.size()){

            while(timeStamp[i] >= lowerBound && timeStamp[i] <= upperBound){ 
                avgRange += energyInMatch[i];
                i++;
                n++;
            }

            timeInterval.push_back(lowerBound); 
            avgEnergyInterval.push_back(avgRange/n);

            lowerBound = upperBound;
            upperBound += BrownoutModuleRunInterval/1000.0;
            n = 0;
            
        }
        
    }

    //read in all past match data
    sumAllMatchesStream.open(sumAllMatchesFile);

    int numOfMatches;
    double energySum, avg = 0;
    std::vector<double> allMatchesSum;
    sumAllMatchesStream >> numOfMatches;

    // Get average of each point
    for(uint8_t i = 0; i < avgEnergyInterval.size(); i++){
        sumAllMatchesStream >> energySum;
        avg = double(avgEnergyInterval[i] + energySum)/numOfMatches;
        pastEnergy.push_back(avg);
        avg = 0;
        allMatchesSum[i] = avgEnergyInterval[i] + energySum;
    }
    numOfMatches++;

    //clear file then update all match sums
    sumAllMatchesStream.open(sumAllMatchesFile, std::ofstream::out | std::ofstream::trunc);

    //write updated match data to sum of all matches
    for(int i =0; i < allMatchesSum.size(); i++){
        sumAllMatchesStream >> allMatchesSum[i];
    }

    sumAllMatchesStream << numOfMatches;

    sumAllMatchesStream.close();
        
}

/* writes data to csv file */
bool BrownoutModule::writeData(std::string fileName){
    
    if (generalStream <<  frc::Timer().GetFPGATimestamp() << ", " << pdp->GetTotalCurrent() << ", "<< pdp->GetVoltage() << ", " << getBatteryPower() << std::endl){
        
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

bool BrownoutModule::checkEnergy(double time, double matchTime){

    //matchTime counts down for current period; add time to auto 
    if(stateRef->IsAutonomous())
        matchTime += TELEOP_LENGTH;

    allMatchesStream << time << ", "<< matchTime<< "," << pdp->GetTotalEnergy() << std::endl;
    lastMatchStream << time << ", "<< matchTime<< "," << pdp->GetTotalEnergy() << std::endl;

    if(fileEmpty) return false;
    //compare around the same time
    for(uint8_t i = 1; i < pastEnergy.size(); i++){
        if(timeInterval.at(i - 1) <= time && time <= timeInterval.at(i)){
            return (abs(pdp->GetTotalEnergy() - pastEnergy.at(i)) > ENERGY_THRESHOLD*pastEnergy.at(i));
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
    double scaling =  (VOLTAGE_THRESHOLD - pdp->GetVoltage())/(batteryResistance * getMotorCurrentDraw()); 
    scaling *= (100 - nonDriveLoad)/100.0; //scale drive

    if (scaling > 1)
        return 1;

    return scaling;

}

std::vector<uint8_t> BrownoutModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID, DriveBaseModuleID, UpdateMotorModuleID}; }