
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv2/objdetect/objdetect.hpp>
#include </usr/include/opencv2/highgui/highgui.hpp>
#include </usr/include/opencv2/imgproc/imgproc.hpp>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace cv;
	
/****************************************************************/
/*		Method Definition				*/
/****************************************************************/
void analyzeWebCamFeed(float maxSecs);
/* Method to write to an output port */
void writeToPort(Mat img, BufferedPort<ImageOf<PixelRgb> > &port);
/* Edge Detection Implementation*/
Mat detectEdge(Mat src);
/* Object Detection Implementation*/
Mat detectFaceAndCircles(Mat src);
/* Port intialization */
void preparePorts();
/****************************************************************/
/* 		Global variables				*/
/****************************************************************/
double xMeanFace, yMeanFace, xMeanObj, yMeanObj;
int lookCycle, timeLapsed, loopCountFace, loopCountObj;
String 	webCamPort 	= 	"/lap/in/webcam",
	textureScreen 	= 	"/icubSim/texture/screen",
	inputPort	= 	"/lap/in/image1", 		//input port in program
	iCubLeftEye	=	"/icubSim/cam/left",		//left eye of robot
	yarpviewLinear	=	"/lap/view/lfilter",		//yarpview for Linear filtering result
	yarpviewObjectD	=	"/lap/view/odetect",		//yarpview for results of object detection
	outputLinear	= 	"/lap/out/fltrImage",		//Output of Linear filtering 
	outputObjectD	=	"/lap/out/trackedImg",		//Object Detection output
	yarpviewLefteye	= 	"/lap/out/robotlefteye", 	//output from Robot's left eye
	outputXY	= 	"/lap/out/lookhere"; 		//attention coordinates

bool	faceFound;
bool 	objectFound;

BufferedPort<ImageOf<PixelRgb> > imagePort;			//image source port
BufferedPort<ImageOf<PixelRgb> > fltrImagePort;			//Linearfilter's output port
BufferedPort<ImageOf<PixelRgb> > objMapPort;			//output of image tracking result
BufferedPort<yarp::sig::Vector>  objMapXYPort;		


/****************************************************************/
/*		Method Implementations				*/
/****************************************************************/

/****************************************************************/
/*		The Main Method					*/
/****************************************************************/
int main(int argc, char* argv[]) {

	Network yarp; // set up yarp
	preparePorts();
	//keep running .5 seconds analysis
	while(1){
		  analyzeWebCamFeed(0.5); //half a second analyzes
	}
}//end of main

/****************************************************************/
/*		Analyze Web Cam Feed				*/
/****************************************************************/

void analyzeWebCamFeed(float maxSecs){
	
	Mat src, dest; float seconds = 0;
	ImageOf<PixelRgb> *CamImage;
	clock_t start = clock();
	while (1) { // repeat forever
		//exit when N seconds lapse;
		if( maxSecs > 0 ) {
			clock_t elapsed = clock() - start;
			seconds = ((float)elapsed)/CLOCKS_PER_SEC;
			if(seconds > maxSecs){break;} else{printf("/n Time Lapsed %f seconds", seconds);} 
		} //else unlimited loop.
		CamImage = imagePort.read(); //read the image from the port
		//reset the flags
		objectFound = false; faceFound = false;
		if(CamImage!=NULL){ //thats an image
			printf("We got an image of size %dx%d\n", CamImage->width(), CamImage->height());
			//we have a Yarp image here, lets convert it to IPL(Intel Image Processing Library) format, which OpenCV support.
			//First, reserve a pointer of the size of our source image. Force the size
			IplImage *oCVSrc = cvCreateImage(cvSize(CamImage->width(), CamImage->height()),IPL_DEPTH_8U,3);
			//OpenCV assumptions of our color might be wrong, so cast and color convert
			cvCvtColor((IplImage*)CamImage->getIplImage(), oCVSrc, CV_RGB2BGR);
			//convert image to Array (Mat)
			src = cvarrToMat(oCVSrc);
			//apply edge detection 
			dest = detectEdge(src);
			//show the edges in a YarpView.
			ImageOf<PixelRgb> fltrImg;//convert back to Yarpimage
			IplImage tmp = dest;
			fltrImg.wrapIplImage(&tmp);
			printf("writing image to port /lap/out/view2");
			//write to output port
			fltrImagePort.prepare() = fltrImg;
			fltrImagePort.write();
			//detect face and show with an box to the output view
			Mat faceDetectionResult = detectFaceAndCircles(src);
			//cvtColor( faceDetectionResult, faceDetectionResult, CV_BGR2RGB ); 
			writeToPort(faceDetectionResult, objMapPort);
			
		}//end if camImage is NULL	
	}//end of repeat forever loop
	yarp::sig::Vector& v = objMapXYPort.prepare();
	v.resize(3); v[0] = 0; v[1] = 0; v[2] = 0;
	if (faceFound) {	
			v[0] = xMeanFace/loopCountFace; 
			v[1] = yMeanFace/loopCountFace; 
			v[2] = 10; //face
			
	}
	else{ 
			if (objectFound) { 
					v[0] = xMeanObj/loopCountObj; 
					v[1] = yMeanObj/loopCountObj;
					v[2] = 5; //Object
				   }
		 }
	objMapXYPort.write();
	

}
/* function to write images to an output port */
void writeToPort(Mat img, BufferedPort<ImageOf<PixelRgb> > &port){
		ImageOf<PixelRgb> outImg;
		IplImage tmp = img;
		outImg.wrapIplImage(&tmp);
		port.prepare() = outImg;
		port.write();

}
/* function to detect edges */
Mat detectEdge( Mat src){
		Mat dest; Mat gray;
		Mat kernel;
		Point anchor;
		double delta, scale;
		int ddepth;
		int kernel_size;
		// Initialize arguments for the filter
		anchor = Point( -1, -1 ); 
		delta = 0;
		scale = 1;
		ddepth = -1;
		// Apply linear filter or gausian--this we will explore later. to reduce noice
		/* {kernel_size = 5;// using a 5x5 kernel for linear gradient
		kernel = Mat::ones( kernel_size,  kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);
		filter2D(src, dest, ddepth , kernel, anchor, delta, BORDER_DEFAULT );} */
		//GaussianBlur(src, dest, Size(kernel_size, kernel_size), 0, 0, BORDER_DEFAULT);
                GaussianBlur(src, dest, Size(9, 9), 2, 2, BORDER_DEFAULT);
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
		return dest;
}

/****************************************************************/
/*		Method detectFaceAndCircles			*/
/****************************************************************/
Mat detectFaceAndCircles(Mat src){
//this method is to detect the face from the frame, and draw an ellipse around the face.
// when faces are not found, we also look for circles/balls and put a circle around it
		vector<Rect> faces;
		vector<Vec3f> circles;
		Mat gray, gray_hist;
		String face_cascade_name = "haarcascade_frontalface_alt.xml";
		CascadeClassifier face_cascade;
		//intialize variables
		xMeanFace = yMeanFace = xMeanObj = yMeanObj = loopCountFace = loopCountObj = 0;
		//convert the source image to grey scale	
		cvtColor( src, gray, CV_BGR2GRAY ); 
		//equalize the grayscale image to histogram
		/* below function Equalizes the histogram of a grayscale image.
		The function equalizes the histogram of the input image using the following algorithm:
		Calculate the histogram H for src .
		Normalize the histogram so that the sum of histogram bins is 255.
		Compute the integral of the histogram:
		H′i=∑0≤j<iH(j)
		Transform the image using H′ as a look-up table: dst(x,y)=H′(src(x,y))
		The algorithm normalizes the brightness and increases the contrast of the image.
		*/
		equalizeHist( gray, gray_hist );
		 //-- Detect faces using the Cascade classifier
		if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading cascade classifier file\n"); return -1; };
  		 face_cascade.detectMultiScale( gray_hist, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
		if(faces.size() > 0){objectFound = true; faceFound = true;}
		//draw an Rectangle around the face
		for( size_t i = 0; i < faces.size(); i++ )
		  {
		    rectangle(src, Point( faces[i].x , faces[i].y), 
				   Point( faces[i].x + faces[i].width, faces[i].y + faces[i].height*1.25  ), 
				   Scalar( 85, 255, 0 ), 2, CV_AA, 0);
			xMeanFace += faces[i].x; yMeanFace += faces[i].y; loopCountFace += 1;
			//double z = faces[i].z;
		  }
		/*---- During this, we also see if we can detect some circles---just in case a face is not found---*/
		// take the greyscale of our source and use blur to reduce the noise so we avoid a false circle detection
  		GaussianBlur( gray, gray, Size(5, 5), 0, 0 );
		medianBlur(gray, gray, 5);
		//Apply the Hough Transformation to detect circles
		//HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 200, 100, 0, 0 );
		printf("Trying to detect circles\n");
		HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, 20, 130, 30, 0, 0 );
		if(circles.size() > 0){objectFound = true;}
		//draw a circle around the detected circle	
		  for( size_t i = 0; i < circles.size(); i++ )
		  {   printf("Found a circle \n");
		      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		      int radius = cvRound(circles[i][2]);
		      // circle outline 
		      circle( src, center, radius, Scalar(0,155,255), 3, 8, 0 );
		      xMeanObj += circles[i][0]; yMeanObj += circles[i][1]; loopCountObj += 1;	
		   }
		  return src;		
	}


void preparePorts(){

	//connect webcam to texture screen / ?is this needed?
	Network::connect(webCamPort, textureScreen); 
	/*--setup other ports--*/
	// make a port for reading images
	imagePort.open(inputPort); 
	Network::connect(iCubLeftEye,inputPort);
	//connect leftye to viewport
	Network::connect(iCubLeftEye, yarpviewLefteye); 
	//port for Linear output	
	fltrImagePort.open(outputLinear);
	Network::connect(outputLinear, yarpviewLinear);
	// a port for the object/face tracking
	objMapPort.open(outputObjectD);
	Network::connect(outputObjectD, yarpviewObjectD);
	// output for the XY coords of object
	objMapXYPort.open(outputXY);

}

