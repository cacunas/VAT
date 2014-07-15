#include "MorphologyModule.h"
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

MorphologyModule::MorphologyModule(Datapool *i_data):
	ModuleInterface(i_data)//,
//	vibemat()
{
//	TODO: initialize vibemat
}

MorphologyModule::~MorphologyModule() {}

bool MorphologyModule::setParameters(QDomNode& config){
	return true;
}

bool MorphologyModule::updateParameters(){
	return true;
}

bool MorphologyModule::init(){
//	m_data->rFgImage;
//	TODO: call init_model
	return true;
}

bool MorphologyModule::run(){
	QImage *fg = m_data->rFgImage;
	cv::Mat image = qImage2Mat(fg);

	//Rectangular structuring element
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
												 cv::Size( 3, 3 ),
												 cv::Point( 1, 1 ) );

	//	TODO: call fg_vibe

//	m_data->runModules = false;
	return true;
}
