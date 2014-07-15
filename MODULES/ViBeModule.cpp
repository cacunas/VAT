#include "ViBeModule.h"
#include <QImage>

static cv::Mat qImage2Mat(QImage * qImage)
{
//	int width = qImage->width();
	int height = qImage->height();

	void *raw = qImage->bits();
	int bytesPerLine = qImage->bytesPerLine();

	/*!
	// Copiando sería así:
//	cv::Mat f(height, bytesPerLine, CV_8U);
//	memcpy(f.data, raw, height*bytesPerLine);
	// Y después habría que actualizar la imagen inicial así:
//	int height = fg->height();
//	int bytesPerLine = fg->bytesPerLine();
//	uchar * bits = fg->bits();
//	memcpy(bits, image.data, height*bytesPerLine);
*/

	// Pero yo lo hago chantamente linkeandolo con la representación del QImage
	cv::Mat f(height, bytesPerLine, CV_8U, raw);
	return f;
}

ViBeModule::ViBeModule(Datapool *i_data):
	ModuleInterface(i_data),
	lookUpTable(1, 256, CV_8U)
{
	rnd = cv::theRNG();
	N=20;//knob
	R=20;//knob

	#define rndSize 256
	ri=0;
	#define rdx ri++

	noMin=2;//knob
	phi=16;//knob//ghost?//int phi=0;//dint work?pseudo code.oops a case of premature d..bitching

}

ViBeModule::~ViBeModule() {}

bool ViBeModule::setParameters(QDomNode& config){
	return true;
}

bool ViBeModule::updateParameters(){
	return true;
}

bool ViBeModule::init(){
//	ini
	return true;
}

bool ViBeModule::run(){
	QImage *fg = m_data->rFgImage;
	cv::Mat image = qImage2Mat(fg);

	//Rectangular structuring element
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
												 cv::Size( 3, 3 ),
												 cv::Point( 1, 1 ) );

	cv::dilate(image, image, element, cv::Point(-1,-1), 1);
	cv::erode( image, image, element, cv::Point(-1,-1), 1);

//	m_data->runModules = false;
	return true;
}


void ViBeModule::init_vibe()
{
	for (int i = 0; i < 256; ++i)
		table[i] = (uchar)(R * (i/R));

	uchar* p = lookUpTable.data;
	for( int i = 0; i < 256; ++i)
		p[i] = table[i];

	for(int i=0;i<rndSize;i++)
	{
		rndp[i]=rnd(phi);
		rndn[i]=rnd(N);
		rnd8[i]=rnd(8);
	}
}

int ViBeModule::init_model(cv::Mat& firstSample)
{
	std::vector<cv::Mat> channels;
	split(firstSample,channels);
	if(!initDone)
	{
		init_vibe();
		initDone=0;
	}
	model* m=new model;
	m->fgch= new cv::Mat*[channels.size()];
	m->samples=new cv::Mat**[N];
	m->fg=new cv::Mat(cv::Size(firstSample.cols,firstSample.rows), CV_8UC1);
	for(unsigned int s=0;s<channels.size();s++)
	{
		m->fgch[s]=new cv::Mat(cv::Size(firstSample.cols,firstSample.rows), CV_8UC1);
		cv::Mat** samples= new cv::Mat*[N];
		for(int i=0;i<N;i++)
		{
			samples[i]= new cv::Mat(cv::Size(firstSample.cols,firstSample.rows), CV_8UC1);
		}
		for(int i=0;i<channels[s].rows;i++)
		{
			int ioff=channels[s].step.p[0]*i;
			for(int j=0;j<channels[0].cols;j++)
			{
				for(int k=0;k<N;k++)
				{
					(samples[k]->data + ioff)[j]=channels[s].at<uchar>(i,j);
				}
				(m->fgch[s]->data + ioff)[j]=0;

				if(s==0)(m->fg->data + ioff)[j]=0;
			}
		}
		m->samples[s]=samples;
	}
	models.push_back(m);
	return models.size()-1;
}

void ViBeModule::fg_vibe1Ch(cv::Mat& frame,cv::Mat** samples,cv::Mat* fg)
{
	int step=frame.step.p[0];
	//#pragma omp parallel for
	for(int i=1;i<frame.rows-1;i++)
	{
		int ioff= step*i;
		//#pragma omp parallel for
		for(int j=1;j<frame.cols-1;j++)
		{
			int count =0,index=0;
			while((count<noMin) && (index<N))
			{
			//hotspot but may need to untangle the waves somewhere else
				int dist= (samples[index]->data + ioff)[j]
						  - (frame.data + ioff)[j];
				if(dist<=R && dist>=-R)
				{
					count++;
				}
				index++;
			}
			if(count>=noMin)
			{
				((fg->data + ioff))[j]=0;
				int rand= rndp[rdx];
				if(rand==0)
				{
					rand= rndn[rdx];
					(samples[rand]->data + ioff)[j]=(frame.data + ioff)[j];
				}
				rand= rndp[rdx];
				int nxoff=ioff;
				if(rand==0)
				{
//					int nx=i;
					int ny=j;
					int cases= rnd8[rdx];
					switch(cases)
					{
					case 0:
						//nx--;
						nxoff=ioff-step;
						ny--;
						break;
					case 1:
						//nx--;
						nxoff=ioff-step;
//						ny;
						break;
					case 2:
						//nx--;
						nxoff=ioff-step;
						ny++;
						break;
					case 3:
						//nx++;
						nxoff=ioff+step;
						ny--;
						break;
					case 4:
						//nx++;
						nxoff=ioff+step;
//						ny;
						break;
					case 5:
						//nx++;
						nxoff=ioff+step;
						ny++;
						break;
					case 6:
						//nx;
						ny--;
						break;
					case 7:
						//nx;
						ny++;
						break;
					}
					rand= rndn[rdx];
					(samples[rand]->data + nxoff)[ny]=(frame.data + ioff)[j];
				}
			}else
			{
				((fg->data + ioff))[j]=255;
			}
		}
	}
}

cv::Mat* ViBeModule::fg_vibe(cv::Mat& frame,int idx)
{
	std::vector<cv::Mat> channels;
	split(frame,channels);
	//#pragma omp parallel for
	for(unsigned int i=0;i<channels.size();i++)
	{
		LUT(channels[i], lookUpTable, channels[i]);
		fg_vibe1Ch(channels[i],models[idx]->samples[i],models[idx]->fgch[i]);
		if(i>0 && i<2)
		{
			bitwise_or(*models[idx]->fgch[i-1],
					   *models[idx]->fgch[i],*models[idx]->fg);
		}
		if(i>=2)
		{
			bitwise_or(*models[idx]->fg,*models[idx]->fgch[i],*models[idx]->fg);
		}
	}
	if(channels.size()==1) return models[idx]->fgch[0];
	return models[idx]->fg;
}

