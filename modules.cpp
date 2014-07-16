#include "modules.h"
#include "VideoAnalysis.h"

//1. Add the header file
#include "MODULES/AcquisitionCameraModule.h"
#include "MODULES/AcquisitionModule.h"
#include "MODULES/ETISEOAcquisitionModule.h"
#include "MODULES/ContextModule.h"
#include "MODULES/FakeSegmentationModule.h"
#include "MODULES/FakeTrackingModule.h"
#include "MODULES/trackingmodule.h"
#include "MODULES/segmentationmodule.h"
#include "MODULES/ReliabilityTracking2DModule.h"
#include "MODULES/ReliabilityTrackingModule.h"
#include "MODULES/ReliabilityMultiModelTrackingModule.h"
#include "MODULES/BlobFilteringModule.h"
#include "MODULES/BackgroundInitialization.h"
#include "MODULES/ConnectedComponentsTwoStepsModule.h"
#include "MODULES/ConnectedComponentsRLEModule.h"
#include "MODULES/opencvtracking.h"
#include "MODULES/ETISEOOutputModule.h"
#include "MODULES/IncrementalEventLearningModule.h"
#include "MODULES/ObjectFilteringModule.h"
#include "MODULES/PlanarStatisticsModule.h"
#include "MODULES/regionalSegmentationModule.h"
#include "MODULES/HullToObjectModule.h"
#include "MODULES/DeguObjectModule.h"
#include "MODULES/PersonalityModule.h"
#include "MODULES/segmentationWithColorFilterModule.h"
#include "MODULES/MorphologyModule.h"
#include "MODULES/ViBeModule.h"
#include "MODULES/GrayCodeModule.h"
#include "MODULES/HMTrackingModule.h"


//2. Add the module, according to defined class name in the corresponding header file
// (Creates ALLOCATE_MODULE(module) function, which returns a ModuleInterface * to an
// specific module instance)
ADD_MODULE(AcquisitionCameraModule)
ADD_MODULE(AcquisitionModule)
ADD_MODULE(ETISEOAcquisitionModule)
ADD_MODULE(ContextModule)
ADD_MODULE(FakeSegmentationModule)
ADD_MODULE(segmentationModule)
ADD_MODULE(BackgroundInitializationModule)
ADD_MODULE(FakeTrackingModule)
ADD_MODULE(trackingModule)
ADD_MODULE(ReliabilityTracking2DModule)
ADD_MODULE(ReliabilityTrackingModule)
ADD_MODULE(ReliabilityMultiModelTrackingModule)
ADD_MODULE(BlobFilteringModule)
ADD_MODULE(ConnectedComponentsTwoStepsModule)
ADD_MODULE(ConnectedComponentsRLEModule)
ADD_MODULE(OpencvTracking)
ADD_MODULE(ETISEOOutputModule)
ADD_MODULE(IncrementalEventLearningModule)
ADD_MODULE(ObjectFilteringModule)
ADD_MODULE(PlanarStatisticsModule)
ADD_MODULE(regionalSegmentationModule)
ADD_MODULE(HullToObjectModule)
ADD_MODULE(DeguObjectModule)
ADD_MODULE(PersonalityModule)
ADD_MODULE(segmentationWithColorFilterModule)
ADD_MODULE(MorphologyModule)
ADD_MODULE(ViBeModule)
ADD_MODULE(GrayCodeModule)
ADD_MODULE(HMTrackingModule)

//3. Register module: sets name, type (if provided), and associates name to list.
// Available types: ACQUISITION, INITIALIZATION, SEGMENTATION, TRACKING, ANALYSIS, OTHER
void VideoAnalysis::setAvailableModules() {
    REGISTER_MODULE(AcquisitionCameraModule, ACQUISITION);
    REGISTER_MODULE(AcquisitionModule, ACQUISITION);
    REGISTER_MODULE(ETISEOAcquisitionModule, ACQUISITION);
    REGISTER_MODULE(ContextModule, INITIALIZATION);
    REGISTER_MODULE(FakeSegmentationModule, SEGMENTATION);
    REGISTER_MODULE(segmentationModule, SEGMENTATION);
    REGISTER_MODULE(BackgroundInitializationModule, SEGMENTATION);
    REGISTER_MODULE(ConnectedComponentsTwoStepsModule, SEGMENTATION);
    REGISTER_MODULE(ConnectedComponentsRLEModule, SEGMENTATION);
    REGISTER_MODULE(FakeTrackingModule, TRACKING);
    REGISTER_MODULE(trackingModule, TRACKING);
    REGISTER_MODULE(ReliabilityTracking2DModule, TRACKING);
    REGISTER_MODULE(ReliabilityTrackingModule, TRACKING);
    REGISTER_MODULE(ReliabilityMultiModelTrackingModule, TRACKING);
    REGISTER_MODULE(OpencvTracking, TRACKING);
    REGISTER_MODULE(BlobFilteringModule, OTHER);
    REGISTER_MODULE(ETISEOOutputModule, OTHER);
    REGISTER_MODULE(IncrementalEventLearningModule, ANALYSIS);
    REGISTER_MODULE(ObjectFilteringModule, OTHER);
    REGISTER_MODULE(PlanarStatisticsModule, OTHER);
    REGISTER_MODULE(regionalSegmentationModule, SEGMENTATION);
    REGISTER_MODULE(HullToObjectModule, ANALYSIS);
    REGISTER_MODULE(DeguObjectModule, ANALYSIS);
    REGISTER_MODULE(PersonalityModule, OTHER);
    REGISTER_MODULE(segmentationWithColorFilterModule, SEGMENTATION);
	REGISTER_MODULE(MorphologyModule, SEGMENTATION);
	REGISTER_MODULE(ViBeModule, SEGMENTATION);
    REGISTER_MODULE(GrayCodeModule, OTHER);
    REGISTER_MODULE(HMTrackingModule, TRACKING);
}
