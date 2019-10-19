
#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Network.h>
/*#include </usr/include/opencv2/imgproc.hpp>
#include </usr/include/opencv2/imgcodecs.hpp>
#include </usr/include/opencv2/highgui.hpp>*/
#include </usr/include/opencv2/opencv.hpp>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace cv;

int main(int argc, char* argv[]) {

	Network yarp; // set up yarp
	BufferedPort<ImageOf<PixelRgb> > imagePort; // make a port for reading images
	
 	imagePort.open("/lap/in/image1");  // give the port a name
 	
 	Network::connect("/icubSim/cam/left","/lap/in/image1");

	BufferedPort<ImageOf<PixelRgb> > fltrImagePort;
	//BufferedPort<Vector> fltrOutPort;
	fltrImagePort.open("/lap/out/fltrImage");
	Network::connect("/lap/out/fltrImage", "/lap/out/view2");
	Mat src; Mat dest; Mat gray;
    	Mat kernel;
	Point anchor;
	double delta, scale;
	int ddepth;
	int kernel_size;
	//IplImage oCVDest;

	while (1) { // repeat forever
		ImageOf<PixelRgb> *CamImage = imagePort.read(); //read the image from the port
		if(CamImage!=NULL){ //thats an image
		    printf("We got an image of size %dx%d\n", CamImage->width(), CamImage->height());
		    //we have a Yarp image here, lets convert it to IPL(Intel Image Processing Library) format, which OpenCV support.
		    //First, reserve a pointer of the size of our source image. Force the
		    IplImage *oCVSrc = cvCreateImage(cvSize(CamImage->width(), CamImage->height()),IPL_DEPTH_8U,3);
		    //OpenCV assumptions of our color might be wrong, so cast and color convert
		    cvCvtColor((IplImage*)CamImage->getIplImage(), oCVSrc, CV_RGB2BGR);
		    printf("trying filter2D, 100 fold");
		    // Initialize arguments for the filter
		    anchor = Point( -1, -1 ); 
		    delta = 0;
		    scale = 1;
		    ddepth = -1;
		    int kernel_size = 5;// using a 3x3 kernel for linear gradient
		    src = cvarrToMat(oCVSrc);
		    kernel = Mat::ones( kernel_size,  kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);
	            // Apply linear filter or gausian--this we will explore later. to reduce noice
		    //filter2D(src, dest, ddepth , kernel, anchor, delta, BORDER_DEFAULT );
		     GaussianBlur(src, dest, Size(kernel_size, kernel_size), 0, 0, BORDER_DEFAULT);
		    //to grayscale
		    cvtColor(dest, gray, COLOR_BGR2GRAY);
		    //get sobel derivatives in x and y directions
		    Mat grad_x, grad_y;
    	            Mat abs_grad_x, abs_grad_y;
		    kernel_size = 3; //3 x 3 for Sobel function
		    Sobel(gray, grad_x, ddepth, 1, 0, kernel_size, scale, delta, BORDER_DEFAULT);
    		    Sobel(gray, grad_y, ddepth, 0, 1, kernel_size, scale, delta, BORDER_DEFAULT);
		    // get the absolute values .
		    convertScaleAbs(grad_x, abs_grad_x);
		    convertScaleAbs(grad_y, abs_grad_y);
		    // ADD UP X AND Y Gradients to get an approximation
		    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dest);
		//now we have to send this image to Yarpview, so convert back to Yarp img.
		ImageOf<PixelRgb> fltrImg;
		IplImage tmp = dest;
		fltrImg.wrapIplImage(&tmp);
		printf("writing image to port /lap/out/view2");
		//write to output port
		fltrImagePort.prepare() = fltrImg;
		fltrImagePort.write();
		}	
	}

}
