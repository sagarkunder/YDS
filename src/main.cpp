#include "ardrone/ardrone.h"
#include<iostream>
#include<opencv\cv.h>
#include<opencv\highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
using namespace cv;
using namespace std;
int new_counter=0;
static Mat IMAGE_HSV,IMAGE_THRESHOLDED,IMAGE_TEMP,IMAGE_LINES;
double dM01,dM10,dArea;
int LOW_HUE = 170,HIGH_HUE = 179,LOW_SATURATION = 150,HIGH_SATURATION = 255,LOW_VALUE = 60,HIGH_VALUE = 255,option;
static int posX=0,posY=0,iLastX = -1,iLastY= -1,setX,setY;
Moments oMoments;

double new_area=0;
// AR.Drone class
ARDrone ardrone;

int operate_ardrone();
void melissa_benoist();
void Follow_Object();
void selena_gomez();
void Control_Trackbar();
void delay(unsigned int time);
void Alisson_Stokke();

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	//Initialize
    if (!ardrone.open()) {
    std::cout << "Failed to initialize." << std::endl;
    return -1;
    }
    Control_Trackbar();
	operate_ardrone();
    return 0;
}

void Control_Trackbar()
{
	namedWindow("Control window", CV_WINDOW_AUTOSIZE);
	cvCreateTrackbar("LowH", "Control window", &LOW_HUE, 179);
	cvCreateTrackbar("HighH", "Control window", &HIGH_HUE, 179);
    cvCreateTrackbar("LowS", "Control window", &LOW_SATURATION, 255);
    cvCreateTrackbar("HighS", "Control window", &HIGH_SATURATION, 255);
    cvCreateTrackbar("LowV", "Control window", &LOW_VALUE, 255); 
    cvCreateTrackbar("HighV", "Control window", &HIGH_VALUE, 255);
}

void melissa_benoist()
{
	while(1)
	{
		// Key input
        int key = cv::waitKey(33);
        if (key == 0x1b){
			ardrone.landing();
			//break;
		}

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Display the image
		IMAGE_TEMP=image;
        cv::imshow("camera", IMAGE_TEMP);
		IMAGE_LINES = Mat::zeros( IMAGE_TEMP.size(), CV_8UC3 );
		cvtColor(image, IMAGE_HSV, COLOR_BGR2HSV);
		//inRange(IMAGE_HSV, Scalar(LOW_HUE, LOW_SATURATION, LOW_VALUE), Scalar(HIGH_HUE, HIGH_SATURATION, HIGH_VALUE), IMAGE_THRESHOLDED); 
	    inRange(IMAGE_HSV, Scalar(140, 63, 63), Scalar(179, 255, 255), IMAGE_THRESHOLDED); 
        erode(IMAGE_THRESHOLDED, IMAGE_THRESHOLDED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( IMAGE_THRESHOLDED, IMAGE_THRESHOLDED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        dilate( IMAGE_THRESHOLDED, IMAGE_THRESHOLDED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(IMAGE_THRESHOLDED, IMAGE_THRESHOLDED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		//counter for area
        oMoments = moments(IMAGE_THRESHOLDED);
        dM01 = oMoments.m01;
        dM10 = oMoments.m10;
        dArea = oMoments.m00;
		posX = dM10 / dArea;
        posY = dM01 / dArea;
        imshow("Thresholded Image", IMAGE_THRESHOLDED);
	    IMAGE_TEMP = IMAGE_TEMP + IMAGE_LINES;
		cv::circle(IMAGE_TEMP, cv::Point(posX,posY),(dArea/2),cv::Scalar(255,0,0),1,8,0);
		cv::putText(IMAGE_TEMP,"Battery = "+ardrone.getBatteryPercentage(),cv::Point(4,4),1,2,Scalar(255,0,0),1,8,false);
        imshow("View window", IMAGE_TEMP);
        waitKey(33);
		if(((posY<170)&&(posY>0)&&(posY<360))||((posY>190)&&(posY>0)&&(posY<360))||((posX<310)&&(posX>0)&&(posY<640))||((posY<330)&&(posY>0)&&(posY<640))){//||||((dArea<(new_area+new_area*0.2))||((dArea>(new_area-new_area*0.2))))
		if(new_counter==0)
		{
			new_counter++;
			const double setArea=oMoments.m00;
			cout<<"  "<<setArea<<"\n";
			new_area=setArea;
		}
		if(new_counter>0)
		{
			if (dArea > 100000)
	        {
				posX = dM10 / dArea;
                posY = dM01 / dArea;        
				if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
		        {
					line(IMAGE_LINES, Point(posX, posY), Point(iLastX, iLastY), Scalar(255,255,0), 2);
				}
                iLastX = posX;
                iLastY = posY;
			    //cout<<posX<<"\n";		
			    //cout<<iLastX<<"\n";
			    cout<<posX<<"   "<<posY<<"   "<<dArea<<"   "<<new_area<<"\n";
			    //cout<<iLastY<<"\n";
			    cout<<dArea<<"\n";
			}
		}
		delay(3000);
		Follow_Object();
		}
	}
} 

void delay(unsigned int time)
{
	unsigned int i,j;
	for(i=0;i<time;i++)
		for(j=0;j<1565;j++);
}

int operate_ardrone()
{
	cout<<"Select the Mode to control the AR-drone\n1.Manual\n2.Automated (Red coloured object tracking and following)\n\n";
	cin>>option;
	switch(option)
	{
	case 1:
		
		// Battery
        std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

        // Instructions
        std::cout << "***************************************" << std::endl;
        std::cout << "*       CV Drone sample program       *" << std::endl;
        std::cout << "*           - How to play -           *" << std::endl;
        std::cout << "***************************************" << std::endl;
        std::cout << "*                                     *" << std::endl;
        std::cout << "* - Controls -                        *" << std::endl;
        std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
        std::cout << "*    'Up'    -- Move forward          *" << std::endl;
        std::cout << "*    'Down'  -- Move backward         *" << std::endl;
        std::cout << "*    'Left'  -- Turn left             *" << std::endl;
        std::cout << "*    'Right' -- Turn right            *" << std::endl;
        std::cout << "*    'Q'     -- Move upward           *" << std::endl;
        std::cout << "*    'A'     -- Move downward         *" << std::endl;
        std::cout << "*                                     *" << std::endl;
        std::cout << "* - Others -                          *" << std::endl;
        std::cout << "*    'C'     -- Change camera         *" << std::endl;
        std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
        std::cout << "*                                     *" << std::endl;
        std::cout << "***************************************" << std::endl;
	    selena_gomez();
	    ardrone.close();
	    break;

	case 2:
		if (ardrone.onGround()) {ardrone.takeoff();}
		delay(3000);
		melissa_benoist();
		//ardrone.close();
		break;

	default:
		cout<<"Invalid choice";
		break;
	}
    return 0;
}

void Follow_Object()
{
	double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
	int key = cv::waitKey(33);
	if (key == 0x1b) ardrone.landing();
	if((posY>190)&&(posY>0)&&(posY<360))
	{
		ardrone.move3D(0.0, 0.0, -0.15, 0.0);
	    //function_to_move_ardrone_down();
		while((posY>190)&&(posY>0)&&(posY<360))
		{
			cout<<"\nMoving ardrone down [ Current position: "<<posY<<" ]\n";
			Alisson_Stokke();
			if (key == 0x1b) ardrone.landing();
		}
	}
	else
	{
		ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	}
	ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	Alisson_Stokke();

	if((posY<170)&&(posY>0)&&(posY<360))
	{
		ardrone.move3D(0.0, 0.0, 0.15, 0.0);
		//function_to_move_ardrone_up();
		while((posY<170)&&(posY>0)&&(posY<360))
		{
			cout<<"\nMoving ardrone up [ Current position: "<<posY<<" ]\n";
			Alisson_Stokke();	
			if (key == 0x1b) ardrone.landing();
		}
	}
	else
	{
		ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	}
	ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	Alisson_Stokke();
	
	if((posX<310)&&(posX>0)&&(posX<640))
	{
		ardrone.move3D(0.0, 0.15, 0.0, 0.0);
		//function_to_move_ardrone_left();
		while((posX<310)&&(posX>0)&&(posX<640))
		{
			cout<<"\nMoving ardrone left [ Current position: "<<posX<<" ]\n";
			Alisson_Stokke();
			if (key == 0x1b) ardrone.landing();
		}
	}
	else
	{
		ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	}
	ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	Alisson_Stokke();
	
	if((posX>330)&&(posX>0)&&(posX<640))
	{
		ardrone.move3D(0.0, -0.15, 0.0, 0.0);
		//function_to_move_ardrone_right();
		while((posX>330)&&(posX>0)&&(posX<640))
		{
			cout<<"\nMoving ardrone right [ Current position: "<<posX<<" ]\n";
			Alisson_Stokke();
			if (key == 0x1b) ardrone.landing();
		}
	}
	else
	{
		ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	}
	ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	Alisson_Stokke();
		
	if(dArea<(new_area-(new_area*0.2))&&(posX>0)&&(posX<640))
	{
		ardrone.move3D(0.15, 0.0, 0.0, 0.0);
		while(dArea<(new_area-(new_area*0.2))&&(posX>0)&&(posX<640))
		{
			cout<<"\nMoving ardrone forward [ Current Area: "<<dArea<<" ]\n";
			Alisson_Stokke();
			if (key == 0x1b) ardrone.landing();
		}
	}
	else
	{
		ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	}
	ardrone.move3D(0.0, 0.0, 0.0, 0.0);
    Alisson_Stokke();
	
	if(dArea>(new_area+(new_area*0.2))&&(posX>0)&&(posX<640))
	{
		ardrone.move3D(-0.15, 0.0, 0.0, 0.0);
		while(dArea>(new_area+(new_area*0.2))&&(posX>0)&&(posX<640))
		{
            cout<<"\nMoving ardrone backward [ Current Area: "<<dArea<<" ]\n";
			Alisson_Stokke();
			if (key == 0x1b) ardrone.landing();
		}
	}
	else
	{
		ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	}
	ardrone.move3D(0.0, 0.0, 0.0, 0.0);
	Alisson_Stokke();
	if (key == 0x1b) ardrone.landing();
}

void selena_gomez()
{
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  0.5;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -0.5;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  0.5;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -0.5;
        if (key == 'j') vy =  0.5;
        if (key == 'l') vy = -0.5;
        if (key == 'q') vz =  0.5;
        if (key == 'a') vz = -0.5;
        ardrone.move3D(vx, vy, vz, vr);

        // Change camera
        static int mode = 0;
        if (key == 'c') ardrone.setCamera(++mode % 4);

        // Display the image
        cv::imshow("camera", image);
    }
}

void Alisson_Stokke()
{
        cv::Mat image = ardrone.getImage();
		IMAGE_TEMP=image;
		// Display the image
        cv::imshow("camera", IMAGE_TEMP);
		IMAGE_LINES = Mat::zeros( IMAGE_TEMP.size(), CV_8UC3 );
		cvtColor(image, IMAGE_HSV, COLOR_BGR2HSV);
		//inRange(IMAGE_HSV, Scalar(LOW_HUE, LOW_SATURATION, LOW_VALUE), Scalar(HIGH_HUE, HIGH_SATURATION, HIGH_VALUE), IMAGE_THRESHOLDED); 
	    inRange(IMAGE_HSV, Scalar(140, 63, 63), Scalar(179, 255, 255), IMAGE_THRESHOLDED); 
        erode(IMAGE_THRESHOLDED, IMAGE_THRESHOLDED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( IMAGE_THRESHOLDED, IMAGE_THRESHOLDED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        dilate( IMAGE_THRESHOLDED, IMAGE_THRESHOLDED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(IMAGE_THRESHOLDED, IMAGE_THRESHOLDED, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		//counter for area
        oMoments = moments(IMAGE_THRESHOLDED);
        dM01 = oMoments.m01;
        dM10 = oMoments.m10;
        dArea = oMoments.m00;
		posX = dM10 / dArea;
        posY = dM01 / dArea;
        if (dArea > 100000)
	    {
		    posX = dM10 / dArea;
            posY = dM01 / dArea;        
            if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
		    {
			    line(IMAGE_LINES, Point(posX, posY), Point(iLastX, iLastY), Scalar(255,255,0), 2);
		    }
            iLastX = posX;
            iLastY = posY;
			//cout<<posX<<"\n";		
			//cout<<iLastX<<"\n";
			//cout<<posX<<"   "<<posY<<"   "<<dArea<<"   "<<new_area<<"\n";
			//cout<<iLastY<<"\n";
			//cout<<dArea<<"\n";
		}
        imshow("Thresholded Image", IMAGE_THRESHOLDED);
	    IMAGE_TEMP = IMAGE_TEMP + IMAGE_LINES;
		cv::circle(IMAGE_TEMP, cv::Point(posX,posY),(dArea/2),cv::Scalar(255,0,0),1,8,0);
        imshow("View window", IMAGE_TEMP);
        waitKey(33);
}

