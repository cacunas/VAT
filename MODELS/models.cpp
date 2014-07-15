#include "models.h"
#include "VideoAnalysis.h"

//1. Add the header file
#include "Blob2DFromBGSubstractionModel.h"
#include "DeguTrackingModel.h"
#include "LightSourceModel.h"

//2. Add the model, according to defined class name in the corresponding header file
// (Creates ALLOCATE_MODEL(model) function, which returns a ReliabilitySingleModelInterface * to a
// specific model instance)
ADD_MODEL(Blob2DFromBGSubstractionModel)
ADD_MODEL(DeguTrackingModel)
ADD_MODEL(LightSourceModel)



//3. Register model: sets name, and associates name to list.

void VideoAnalysis::setAvailableModels() {
    REGISTER_MODEL(Blob2DFromBGSubstractionModel);
    REGISTER_MODEL(DeguTrackingModel);
    REGISTER_MODEL(LightSourceModel);
}
