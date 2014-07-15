#ifndef MORPHOLOGYMODULE_H
#define MORPHOLOGYMODULE_H

#include "ModuleInterface.h"


class MorphologyModule: public ModuleInterface {
public:
	MorphologyModule(Datapool *i_data);
	~MorphologyModule();

	//Set module configuration parameters
	bool setParameters(QDomNode& config);

	//Initialization after reading parameters
	bool init();

	//Function executed at each frame
	bool run();

	//update parameters at runtime.
	bool updateParameters();
};

#endif // MORPHOLOGYMODULE_H
