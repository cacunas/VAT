#include "BackgroundInitialization.h"
#include <errno.h>
#include <iostream>
#include "image_display.h"
#include "src/MathFunctions.h"
#include <QMessageBox>
#include <QPainter>
#include <QFile>
#include <QTextStream>


BackgroundInitializationModule::BackgroundInitializationModule(Datapool *i_data):ModuleInterface(i_data)
{
  firstTime = true;
	initializationStage = true;
}

BackgroundInitializationModule::~BackgroundInitializationModule(){}

bool BackgroundInitializationModule::setParameters(QDomNode& config)
{
	QDomNode n;
	if(config.isNull())  //Parameter set for module not defined
	{
		this->sizeBlock = 10;
		this->thresholdStill = 20;
		this->aggregationRate = 0.95;
		this->thresholdMov = 0.9 ;
		this->thresholdCorrelation = 0.7;
	}
	else
	{
		if( !( n = XmlCommon::XmlCommon::getParameterNode("sizeBlock", config) ).isNull() )
				this->sizeBlock = XmlCommon::getParameterValue(n).toInt();
		else
		{
			this->sizeBlock = 10;
            AppendToLog("BackgroundInitializationModule: Warning: 'sizeBlock' not defined. Taking defaults: \n\t\tsizeBlock = " + QString::number(this->sizeBlock) + "\n");
		}
		if( !( n = XmlCommon::XmlCommon::getParameterNode("thresholdStill", config) ).isNull() )
			this->thresholdStill = XmlCommon::getParameterValue(n).toInt();
		else
		{
			this->thresholdStill = 20;
            AppendToLog("BackgroundInitializationModule: Warning: 'thresholdStill' not defined. Taking defaults: \n\t\tthresholdStill = " + QString::number(this->thresholdStill) + "\n");
		}
		if( !( n = XmlCommon::XmlCommon::getParameterNode("aggregationRate", config) ).isNull() )
			this->aggregationRate = XmlCommon::getParameterValue(n).toFloat();
		else
		{
			this->aggregationRate = 0.95;
            AppendToLog("BackgroundInitializationModule: Warning: 'aggregationRate' not defined. Taking defaults: \n\t\taggregationRate = " + QString::number(this->aggregationRate) + "\n");
		}
		if( !( n = XmlCommon::XmlCommon::getParameterNode("thresholdMov", config) ).isNull() )
			this->thresholdMov = XmlCommon::getParameterValue(n).toFloat();
		else
		{
			this->thresholdMov = 0.9;
            AppendToLog("BackgroundInitializationModule: Warning: 'thresholdMov' not defined. Taking defaults: \n\t\tthresholdMov = " + QString::number(this->thresholdMov) + "\n");
		}
		if( !( n = XmlCommon::XmlCommon::getParameterNode("thresholdCorrelation", config) ).isNull() )
			this->thresholdCorrelation = XmlCommon::getParameterValue(n).toFloat();
		else
		{
			this->thresholdCorrelation = 0.7;
            AppendToLog("BackgroundInitializationModule: Warning: 'thresholdCorrelation' not defined. Taking defaults: \n\t\tthresholdCorrelation = " + QString::number(this->thresholdCorrelation) + "\n");
		}
	}

	//Setea la lista de parametros y la lista de tipos de parametros utilizadas en el ParameterDialog.
	addParameter("sizeBlock", QString::number(this->sizeBlock), "int");
	addParameter("thresholdStill", QString::number(this->thresholdStill), "int");
	addParameter("aggregationRate", QString::number(this->aggregationRate), "float");
	addParameter("thresholdMov", QString::number(this->thresholdMov), "float");
	addParameter("thresholdCorrelation", QString::number(this->thresholdCorrelation), "float");
	return true;
}

bool BackgroundInitializationModule::init()
{
  return true;
}

bool BackgroundInitializationModule::updateParameters()
{
	parameter *sizeBlock, *thresholdStill, *aggregationRate, *thresholdMov, *thresholdCorrelation;

	sizeBlock = getParameter("sizeBlock");
	thresholdStill = getParameter("thresholdStill");
	aggregationRate = getParameter("aggregationRate");
	thresholdMov = getParameter("thresholdMov");
	thresholdCorrelation = getParameter("thresholdCorrelation");

	this->sizeBlock = sizeBlock->value.toInt();
	this->thresholdStill = thresholdStill->value.toInt();
	this->aggregationRate = aggregationRate->value.toDouble();
	this->thresholdMov = thresholdMov->value.toDouble();
	this->thresholdCorrelation = thresholdCorrelation->value.toDouble();

	return true;
}


bool BackgroundInitializationModule::run()
{

	if(m_data->currentImage == NULL || m_data->currentImage->isNull())
	{
        AppendToLog("BackgroundInitializationModule: Warning: No current image. Aborting execution...\n");
		return false;
	}

	uchar *ptr_finalBackground, *ptr_imgData = m_data->currentImage->bits();

  int imgWidth = m_data->currentImage->width(),
      imgHeight = m_data->currentImage->height();
	cv::Mat imgARGB32 = cv::Mat(imgHeight, imgWidth, CV_8UC4 );

	if( imgWidth % sizeBlock != 0 ||  imgHeight % sizeBlock != 0)
	{
        AppendToLog("BackgroundInitializationModule: Warning: Size of block must divide the image width and height perfectly. Aborting execution...\n");
		return false;
	}

  if(firstTime)
  {
    firstTime = false;
    if(m_data->bgImage == NULL)
    {
			m_data->bgImage= new QImage(m_data->currentImage->width(),m_data->currentImage->height(), QImage::Format_RGB888); //Set Background
			background = cv::Mat(m_data->bgImage->height(),m_data->bgImage->width(),  CV_8UC3, cv::Scalar(UNDEFINED,UNDEFINED,UNDEFINED));
    }

		numBlockUndef = ( m_data->currentImage->width()*m_data->currentImage->height() ) / ( sizeBlock*sizeBlock );
    if(currRepresentation.empty())
			currRepresentation = cv::Mat(m_data->currentImage->height()/sizeBlock, m_data->currentImage->width()/sizeBlock,
                                 CV_8UC1, cv::Scalar(UNDEFINED));
    if(currentFrameC3.empty())
			currentFrameC3 = cv::Mat(imgHeight, imgWidth, CV_8UC3 );

		memcpy(imgARGB32.data, ptr_imgData, imgHeight*m_data->currentImage->bytesPerLine());
		cv::cvtColor(imgARGB32, currentFrameC3, CV_RGBA2BGR);
    cv::cvtColor(currentFrameC3, currentFrameC1, CV_BGR2GRAY );

    return true;
  }
  else
  {
    currentFrameC1.copyTo( previousFrameC1 );
    currRepresentation.copyTo( prevRepresentation );

		memcpy(imgARGB32.data, ptr_imgData, imgHeight*m_data->currentImage->bytesPerLine());
		cv::cvtColor(imgARGB32, currentFrameC3, CV_RGBA2BGR);
		cv::cvtColor(currentFrameC3, currentFrameC1, CV_BGR2GRAY );
  }

  blockMatching(imgWidth, imgHeight);
  blockClassification(imgWidth, imgHeight);
  blockUpdate(imgWidth, imgHeight);

	//copy initialization background to m_data
	ptr_finalBackground = m_data->bgImage->bits();
	memcpy(ptr_finalBackground, this->background.data, this->background.rows*this->background.step);

//	AppendToLog("BackgroundInitializationModule: Bloques por inicializar:" + QString::number(this->numBlockUndef));
	if(this->numBlockUndef == 0)
	{
		this->initializationStage = false; //the end of initialization process
//		AppendToLog("BackgroundInitializationModule: Process finished. \n");
	}

  return true;
}

bool BackgroundInitializationModule::isMovingBlock(float SADcenter, float minSADblock)
{
	if( minSADblock / SADcenter < thresholdMov )
    return true;
  else
    return false;
}

//slide window, using the previous block as pattern and blocks from current frame for search space
void BackgroundInitializationModule::blockMatching(int imgWidth, int imgHeight)
{
  uchar *ptr_currRepresentation = currRepresentation.data;
  float * ptr_matchTemplateResult;
	int i, j, pos, infCol, supCol, infRow, supRow, patternLoc;
	double posPatternRow, posPatternCol, SADminValue, SADmaxValue; //SAD: sum absolute difference
	cv::Mat patternBlock, searchSpace, matchTemplateResult;
	cv::Point SADminLoc, SADmaxLoc;

  pos = 0;
	for(j=0; j<imgHeight - sizeBlock + 1; j += sizeBlock)
  {
		for(i=0; i<imgWidth - sizeBlock + 1; i += sizeBlock)
    {
			posPatternRow = MID;
			posPatternCol = MID;
      infCol = i - (sizeBlock/2);
      infRow = j - (sizeBlock/2);
      supCol = i + 3*(sizeBlock/2);
      supRow = j + 3*(sizeBlock/2);

			//set bound for search space
      if(infCol < 0)
      {
        infCol = 0;
        posPatternCol = INF;
      }
      else
        if(supCol >= imgWidth)
          {
            supCol = imgWidth;
            posPatternCol = SUP;
          }

      if(infRow < 0)
      {
        infRow = 0;
        posPatternRow = INF;
      }
      else
				if(supRow >= imgHeight)
          {
						supRow = imgHeight;
            posPatternRow = SUP;
          }

			patternBlock = previousFrameC1(cv::Range(j, j + sizeBlock ), cv::Range(i, i + sizeBlock ) ) ;
      searchSpace = currentFrameC1(cv::Range(infRow, supRow), cv::Range(infCol, supCol) ) ;

      cv::matchTemplate( searchSpace, patternBlock, matchTemplateResult, CV_TM_SQDIFF);
      ptr_matchTemplateResult = matchTemplateResult.ptr<float>(0);
			cv::minMaxLoc( matchTemplateResult, &SADminValue, &SADmaxValue, &SADminLoc, &SADmaxLoc );
      patternLoc = (matchTemplateResult.cols - 1)*posPatternCol + (matchTemplateResult.rows - 1)*posPatternRow*matchTemplateResult.step;

			//first classification: moving or static
			if(( (unsigned int) patternLoc != SADminLoc.x + SADminLoc.y*matchTemplateResult.step )&& isMovingBlock( ptr_matchTemplateResult[patternLoc], SADminValue ) )
        ptr_currRepresentation[pos] = MOVING ;
      else
        ptr_currRepresentation[pos] = STATIC ;

      pos ++;
    }
  }
}

bool  BackgroundInitializationModule::isStillObj(uchar value)
{
	if(value >= STILL_OBJ && value < ILLUM_CHANGE)
    return true;
  else
    return false;
}

bool  BackgroundInitializationModule::isMovingObj(uchar value)
{
	if(value >= MOVING_OBJ && value < MOVING )
    return true;
  else
    return false;
}

void BackgroundInitializationModule::blockClassification(int imgWidth, int imgHeight)
{
  uchar *ptr_currRepresentation = this->currRepresentation.data,
				*ptr_prevRepresentation = this->prevRepresentation.data;
  int step = this->currRepresentation.step, pos;
  double corrCoeffVal;
  cv::Mat blockBg, blockImg;

	for(int j = 0; j < imgHeight/sizeBlock; j ++)
  {
    for(int i = 0; i < imgWidth/sizeBlock; i++)
    {
      pos = i + j*step;
      if( ptr_prevRepresentation[pos] == UNDEFINED && ptr_currRepresentation[pos] == MOVING )
      {
        ptr_currRepresentation[pos] = UNDEFINED;
      }
			else //it can be update so it needs the second type classification
      {

				blockImg = currentFrameC3(cv::Range(j*sizeBlock, (j+1)*sizeBlock), cv::Range(i*sizeBlock , (i+1)*sizeBlock) ) ;
				blockBg = background(cv::Range(j*sizeBlock, (j+1)*sizeBlock), cv::Range(i*sizeBlock , (i+1)*sizeBlock) ) ;

        corrCoeffVal = corrCoeff( blockImg.data, blockBg.data );

        //segun el coeficiente y la representacion previa, clasificarlo en alguna de las 4 segundas classes

        if( ptr_currRepresentation[pos] == STATIC)
        {
          if(corrCoeffVal > thresholdCorrelation )
          {
						ptr_currRepresentation[pos] = BACKGROUND;
          }
					else //STILL_OBJ
          {
						if( isStillObj (ptr_prevRepresentation[pos]) )
						{ //use the matrix representation to count the number of frame classificated as still
							ptr_currRepresentation[pos] = (ptr_prevRepresentation[pos] + 1 > STILL_OBJ + thresholdStill)?
								STILL_OBJ + thresholdStill: ptr_prevRepresentation[pos] + 1 ;
            }
            else
            {
							ptr_currRepresentation[pos] = STILL_OBJ;
            }

          }
        }
        else //MOVING
        {
					if(corrCoeffVal < thresholdCorrelation )
          {
            //MOVING_OBJ
            if( isMovingObj(ptr_prevRepresentation[pos]) )
            {
              ptr_currRepresentation[pos] = (ptr_prevRepresentation[pos] + 1 > MOVING_OBJ + 1)?
                 MOVING_OBJ + 1: ptr_prevRepresentation[pos] + 1 ;
            }
            else
            {
              ptr_currRepresentation[pos] = MOVING_OBJ;
            }
          }
          else
          {
            ptr_currRepresentation[pos] = ILLUM_CHANGE;
          }
        }

      }
    }
  }
}

double BackgroundInitializationModule::corrCoeff(uchar *ptr_img, uchar *ptr_bg)
{
  double s_xx = 0.0, s_xy = 0.0, s_yy = 0.0, mean_x = 0.0, mean_y = 0.0, diff_x, diff_y,corrCoeffVal;
  int numElem = sizeBlock*sizeBlock*background.channels();

  //mean
  for(int i = 0; i < numElem; i++ )
  {
    mean_x += ptr_img[i];
		mean_y += ptr_bg[i];
  }
  mean_x /= numElem;
  mean_y /= numElem;

  for(int i = 0; i < numElem; i++ )
  {
    diff_x = ptr_img[i] - mean_x;
    diff_y = ptr_bg[i] - mean_y;
    s_xx += diff_x*diff_x;
    s_yy += diff_y*diff_y;
    s_xy += diff_x*diff_y;
  }
  corrCoeffVal = s_xx / sqrt( s_xx * s_yy );

	return abs(corrCoeffVal);
}


double BackgroundInitializationModule::sideMatch(cv::Mat ROI, cv::Mat block, bool isBlockImg)
{
	double SM_metric = 0.0;
	cv::Mat backup, vector;
	uchar *ptr_vector;
	cv::Size bgDimension;
	cv::Point ROIloc;
	bool growUp, growLeft, growRight, growBottom;

	if(isBlockImg)
	{
		ROI.copyTo(backup);
		block.copyTo(ROI);
	}

	ROI.locateROI(bgDimension, ROIloc);

	growUp = (ROIloc.y - 1 < 0)? false: true ;
	growLeft = (ROIloc.x - 1 < 0)? false: true ;
	growRight = (ROIloc.x + sizeBlock + 1 >= bgDimension.width)? false: true ;
	growBottom = (ROIloc.y + sizeBlock + 1  >= bgDimension.width)? false: true ;

	ROI.adjustROI(1,1,1,1); //grow 1 pixel in every direction
	if(growUp)
	{
		vector = block.row(0) - block.row(1);
		vector = vector.mul(vector);
		ptr_vector = vector.data;

		for(int i = 0; i < vector.rows*vector.cols; i++)
			SM_metric += ptr_vector[i];
	}
	if(growLeft)
	{
		vector = block.row(0) - block.row(1);
		vector = vector.mul(vector);
		ptr_vector = vector.data;

		for(int i = 0; i < vector.rows*vector.cols; i++)
			SM_metric += ptr_vector[i];
	}
	if(growRight)
	{
		vector = block.row(0) - block.row(1);
		vector = vector.mul(vector);
		ptr_vector = vector.data;

		for(int i = 0; i < vector.rows*vector.cols; i++)
			SM_metric += ptr_vector[i];
	}
	if(growBottom)
	{
		vector = block.row(0) - block.row(1);
		vector = vector.mul(vector);
		ptr_vector = vector.data;

		for(int i = 0; i < vector.rows*vector.cols; i++)
			SM_metric += ptr_vector[i];
	}


	if(isBlockImg)
		backup.copyTo(ROI);

	return SM_metric;
}

void BackgroundInitializationModule::blockUpdate(int width, int height)
{
	uchar *ptr_currRepresentation = this->currRepresentation.data,
				*ptr_prevRepresentation = this->prevRepresentation.data;
	cv::Mat blockImg, blockBg;

	int pos, step = this->currRepresentation.step;
	double SMimage, SMbackground; //SM : Side-match measure

	for(int j = 0; j < height/sizeBlock; j++)
	{
		for(int i = 0; i < width/sizeBlock; i++)
		{
			pos = i + j*step;

			blockImg = currentFrameC3(cv::Range(j*sizeBlock, (j+1)*sizeBlock), cv::Range(i*sizeBlock , (i+1)*sizeBlock) );
			blockBg = background(cv::Range(j*sizeBlock, (j+1)*sizeBlock), cv::Range(i*sizeBlock , (i+1)*sizeBlock) );

			if( ptr_currRepresentation[pos] == UNDEFINED)
			{
				continue;
			}
			else
			{

				if( ptr_prevRepresentation[pos] == UNDEFINED && (ptr_currRepresentation[pos] == BACKGROUND || ptr_currRepresentation[pos] == STILL_OBJ) )
				{
					blockImg.copyTo( blockBg );
					this->numBlockUndef--;
				}
				else
				{
					if( ptr_currRepresentation[pos] == BACKGROUND || ptr_currRepresentation[pos] == ILLUM_CHANGE)
					{
						//Update using "wiener filter"
						addWeighted( blockBg, aggregationRate, blockImg, 1 - aggregationRate, 0.0, blockBg);
					}
					if( isStillObj(ptr_currRepresentation[pos]) )
					{
						if( ptr_currRepresentation[pos] >= STILL_OBJ +  thresholdStill)
						{//incorporate the current block to the background
						blockImg.copyTo( blockBg );
						}
						//else: maintain the current background
					}

					if(ptr_currRepresentation[pos] == MOVING_OBJ + 1  ) //update MOVING_OBJ
					{
						SMimage = sideMatch(blockBg, blockImg, true);
						SMbackground = sideMatch(blockBg, blockBg, false);
						if( SMimage < SMbackground )
						{//incorporate the current block to the background
							blockImg.copyTo( blockBg );
						}
						//else: maintain the current background
					}
				}
			}
		}
	}

}

