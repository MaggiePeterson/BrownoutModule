// Krishna Mani FRC 2020

#include "Constructor.h"
/*
constructThreadedRobot() maps (a) => std::thread(a->init()) over a set of ModuleBase subclasses, using compile time polymorphism
Constructor's constructor must create a set of std::function<SubClass*> objects that have been passed correctly created pointers to GenericPipes

To make your subclass of ModuleBase able to placed and routed in the graph, initialize the function getConstructorArgs to return a std::vector<uint8_t> constructorArgs = {moduleID1, moduleID2, moduleID3, etc.};
The constructor will fail if you say you have a connection to a module, but that module does not say it has a connection to you
Self-referential connections are disallowed
*/

bool Constructor::constructThreadedRobot(std::vector<ModuleBase*> modules, Robot* stateRef) {
  std::cout << "Construct " << modules.size() << " modules:" << std::endl;

  std::vector< std::vector<GenericPipe*> > pipes = {{ nullptr }};

  // Prepare the size of the vectors
  pipes.resize(modules.size());

  // Prepare each vector to its final size
  for (uint8_t i = 0; i < pipes.size(); i++) {
      pipes[i].resize(modules[i]->getConstructorArgs().size());
  }

  for (uint8_t i = 0; i < modules.size(); i++) {
    ModuleBase* thisModule = modules[i];
    std::vector<uint8_t> thisArg = thisModule->getConstructorArgs();

    std::cout << "Module: " << (int)i << std::endl;
    std::cout << "  Number of Connections: " << thisArg.size() << std::endl;

    for (uint8_t j = 0; j < thisArg.size(); j++) {
      std::cout << "    Pipe: " << (int)j << std::endl;			

      if (pipes[i][j]) { // Only initialize a pipe that is not constructed 
        std::cout << "      (previously constructed)" << std::endl;
        continue;
      }

      uint8_t reference = thisArg[j]; // What module this argument refers to

      if (reference == i) { // Fail on self-referential connections
        std::cout << "      Pipe is self referential\nWeb Construction Failure!" << std::endl;

        return false; 
      } 

      std::cout << "      Refers to: " << (int)reference << std::endl;	

      std::vector<uint8_t> targetArg = modules[reference]->getConstructorArgs(); // The arguments of the module that this module is referring to
      // In order for the pipe to exist, the target module must also list this module in its constructor args
      // We must know at what position this module is listed, so we know the index to pass this pipe to it
      
      uint8_t positionInTarget;

      for (uint8_t k = 0; k < targetArg.size(); k++) {
        if (i == targetArg[k] && !(pipes[reference][k])) { positionInTarget = k; break; } // What index is the reference at
        if (k == (targetArg.size() - 1)) { 
          std::cout << "      Not found in target: module " << (int)reference << "\nWeb Construction Failure!" << std::endl;
          return false;
        }	
      }

      std::cout << "      Found: (module " << (int)reference << ") at position: " << (int)positionInTarget << std::endl;

      pipes[i][j] = pipes[reference][positionInTarget] = new GenericPipe(); // Pass them the same pipe
    }
  }

  // For each element of the vector<ModuleBase*>, create a new thread for the function ModuleBase::init and pass it the correct pipes

  for (uint8_t i = 0; i < modules.size(); i++) { std::thread(&ModuleBase::init, modules[i], pipes[i], stateRef).detach(); }
  return true;
}
