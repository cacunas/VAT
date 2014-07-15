#ifndef VIBEMODULE_H
#define VIBEMODULE_H

#include "ModuleInterface.h"
class ViBeModule : public ModuleInterface {
public:
	ViBeModule(Datapool *i_data);
	~ViBeModule();

	//Set module configuration parameters
	bool setParameters(QDomNode& config);

	//Initialization after reading parameters
	bool init();

	//Function executed at each frame
	bool run();

	//update parameters at runtime.
	bool updateParameters();

//	cv::Mat vibemat;
	//#include "opencv2/core/core.hpp"
	//#include "opencv2/imgproc/imgproc.hpp"
	//#include "opencv2/highgui/highgui.hpp"

	int N;
	int R;

	#define rndSize 256
	unsigned char ri;
	#define rdx ri++

	int noMin;
	int phi;
	cv::RNG rnd;

	struct model {
		cv::Mat*** samples;
		cv::Mat** fgch;
		cv::Mat* fg;
	};

	int rndp[rndSize],rndn[rndSize],rnd8[rndSize];
	std::vector<model*> models;
	int initDone;

	uchar table[256];
	cv::Mat lookUpTable;

	void init_vibe();
	int init_model(cv::Mat& firstSample);
	cv::Mat* fg_vibe(cv::Mat& frame,int idx);
	void fg_vibe1Ch(cv::Mat& frame,cv::Mat** samples,cv::Mat* fg);

};

#endif // VIBEMODULE_H
