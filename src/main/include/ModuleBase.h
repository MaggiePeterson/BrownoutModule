// Krishna Mani FRC 2020 : Base class for all robot threaded modules

#ifndef MODULEBASE_H
#define MODULEBASE_H

#include <vector>
#include <queue>
#include <thread>
#include <chrono>

#include "GenericPipe.h"
#include "Robot.h"

/**
 * Abstract Class ModuleBase, inherit from this to create a module
 * Note: This class cannot be instantiated directly; this is by design
 * Note: Do not implement a constructor; the only place where doing init stuff is guaranteed to be safe is periodicInit() 
 */

class ModuleBase {
	public:
	std::vector<GenericPipe*> pipes; // Refer to: https://github.com/Saint-Francis-Robotics-Team2367/pipes
	std::queue<Message*> errors; // It's your responsibility to report these reasonably

	int msInterval; 
	Robot* stateRef;

	void init(std::vector<GenericPipe*>, Robot*); // Init is implemented as it is common between modules
	
	// Pure Virtual Functions; these are two functions that you implement when you subclass this
	virtual std::vector<uint8_t> getConstructorArgs() = 0;
	virtual void periodicRoutine() = 0;
	virtual void periodicInit() = 0;

};

#endif
