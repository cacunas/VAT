#include "draw.h"
#include "VideoAnalysis.h"
#include "paintView.h"

//1. Add the header file
#include "DRAW/setBlobs.h"
#include "DRAW/setLearning.h"
#include "DRAW/setObjects.h"
#include "DRAW/setMultiModelObjects.h"
#include "DRAW/setMultiModelObjectBuffers.h"
#include "DRAW/setMultiModelObjectBBoxes.h"
#include "DRAW/setMultiModelObjectBBoxProjection.h"
#include "DRAW/setMultiModelObjectCurrentInvolvedArea.h"
#include "DRAW/setCurrent.h"
#include "DRAW/setForeground.h"
#include "DRAW/setBackground.h"
#include "DRAW/setHomographyContext.h"
#include "DRAW/set2DContext.h"
#include "DRAW/setColouredForeground.h"
#include "DRAW/setObjectHulls.h"
#include "DRAW/setDegu.h"
#include "DRAW/setPersonality.h"
#include "DRAW/setPersonalityPCAImage.h"
#include "DRAW/setMultiModelObjectTrajectory2D.h"
#include "DRAW/setGrayCode.h"
#include "DRAW/setHistoStat.h"

//2. Add the draw, according to defined class name in the corresponding header file
// (Creates ALLOCATE_DRAW(draw) function, which returns a drawInterface * to an
// specific draw instance)
ADD_DRAW(setBlobs)
ADD_DRAW(setLearning)
ADD_DRAW(setObjects)
ADD_DRAW(setCurrent)
ADD_DRAW(setForeground)
ADD_DRAW(setBackground)
ADD_DRAW(setHomographyContext)
ADD_DRAW(set2DContext)
ADD_DRAW(setColouredForeground)
ADD_DRAW(setObjectHulls)
ADD_DRAW(setDegu)
ADD_DRAW(setPersonality)
ADD_DRAW(setPersonalityPCAImage)
ADD_DRAW(setMultiModelObjects)
ADD_DRAW(setMultiModelObjectBuffers)
ADD_DRAW(setMultiModelObjectBBoxes)
ADD_DRAW(setMultiModelObjectBBoxProjection)
ADD_DRAW(setMultiModelObjectTrajectory2D)
ADD_DRAW(setMultiModelObjectCurrentInvolvedArea)
ADD_DRAW(setGrayCode)
//TEMP
//ADD_DRAW(setHistoStat)

//3. Register module: sets name and associates name to list.
void paintView::setDraw(){
    REGISTER_DRAW_PAINT(setBlobs);
    REGISTER_DRAW_PAINT(setLearning);
    REGISTER_DRAW_PAINT(setObjects);
    REGISTER_DRAW_PAINT(setCurrent);
    REGISTER_DRAW_PAINT(setForeground);
    REGISTER_DRAW_PAINT(setBackground);
    REGISTER_DRAW_PAINT(setHomographyContext);
    REGISTER_DRAW_PAINT(set2DContext);
    REGISTER_DRAW_PAINT(setColouredForeground);
    REGISTER_DRAW_PAINT(setObjectHulls);
    REGISTER_DRAW_PAINT(setDegu);
    REGISTER_DRAW_PAINT(setPersonality);
    REGISTER_DRAW_PAINT(setPersonalityPCAImage);
    REGISTER_DRAW_PAINT(setMultiModelObjects);
    REGISTER_DRAW_PAINT(setMultiModelObjectBuffers);
    REGISTER_DRAW_PAINT(setMultiModelObjectBBoxes);
    REGISTER_DRAW_PAINT(setMultiModelObjectBBoxProjection);
    REGISTER_DRAW_PAINT(setMultiModelObjectTrajectory2D);
    REGISTER_DRAW_PAINT(setMultiModelObjectCurrentInvolvedArea);
    REGISTER_DRAW_PAINT(setGrayCode);
//TEMP
//REGISTER_DRAW_PAINT(setHistoStat);
}

