#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

void suppr_pts_blanc ( cv::Mat * channel, int seuil)
{
for (int i = 0; i < channel[2].rows;i++)
{
	for (int j = 0; j < channel[2].cols ; j++)
	{
		if ( channel[2].at<unsigned char>(i,j) > seuil && channel[1].at<unsigned char>(i,j) > seuil && channel[0].at<unsigned char>(i,j) > seuil) 
		{
			channel[0].at<unsigned char>(i,j) = 0;
			channel[1].at<unsigned char>(i,j) = 0;
			channel[2].at<unsigned char>(i,j) = 0;
		}
	}
}

}

 int main( int argc, char** argv )
 {
	cv::VideoCapture cap("video/2016-01-26-114234.webm"); 
	//cv::VideoCapture cap("/home/valentin/Bureau/projet/calibration5/2016-01-26-113909.webm"); 

    if ( !cap.isOpened() ) 
    {
         std::cout << "Cannot open the web cam" << std::endl;
         return -1;
    }

   cv::Mat imgOriginal;
	int min = 255;
	int max = 255;
	int nbpts = 0;
	int seuil = 200;
	cv::namedWindow("Original", CV_WINDOW_NORMAL);	
	cv::moveWindow("Original",10,10);
	cv::resizeWindow("Original", 400, 300);
	cv::namedWindow("red", CV_WINDOW_NORMAL);	
	cv::moveWindow("red",440,10);
	cv::resizeWindow("red", 400, 300);
	
	cv::namedWindow("white", CV_WINDOW_NORMAL);
	cv::moveWindow("white",840,10);		
	cv::resizeWindow("white", 400, 300);

	cv::namedWindow("blue", CV_WINDOW_NORMAL);
	cv::moveWindow("blue",440,440);		
	cv::resizeWindow("blue", 400, 300);
    while (true)
    {
        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             std::cout << "Cannot read a frame from video stream" << std::endl;
             break;
        }
 
	cv::imshow("Original", imgOriginal); //show the original image
   // Create Matrices (make sure there is an image in input!)
    //attention format BGR
    cv::Mat img_red;
    cv::Mat channel_red[3];
 
 	cv::split(imgOriginal, channel_red);
 	//les pts blancs sont supprimer pour reduire le "bruit" du ciel
	//suppr_pts_blanc(channel_red, seuil);
		
	//seuillage sur le rouge : on garde que les pixel entre min et max
	cv::inRange(channel_red[2], cv::Scalar(min), cv::Scalar(max), channel_red[2]);
	//mise a 0 de la composante que l'on veut
	channel_red[0]=cv::Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC1);//960 x 1280
	channel_red[1]=cv::Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC1);
	nbpts = 0;
	
	
	//on reunit les composantes ds une img couleur
	cv::merge(channel_red,3,img_red);	
	cv::imshow("red",img_red);
	
	 cv::Mat img_w;
 	 cv::Mat channel_w[3];
	 cv::split(imgOriginal, channel_w);
	/*
	cv::inRange(channel_w[0], cv::Scalar(min), cv::Scalar(max), channel_w[0]);
	cv::inRange(channel_w[1], cv::Scalar(min), cv::Scalar(max), channel_w[1]);
	cv::inRange(channel_w[2], cv::Scalar(min), cv::Scalar(max), channel_w[2]);
/*	 cv::imshow("b",channel_w[0]);
	 cv::imshow("g",channel_w[1]);
	 cv::imshow("r",channel_w[2]);
	*/ 
	
	cv::inRange(channel_w[1], cv::Scalar(min), cv::Scalar(max), channel_w[1]);
	//cv::inRange(channel_w[0], cv::Scalar(min), cv::Scalar(max), channel_w[0]);
	//cv::inRange(channel_w[2], cv::Scalar(min), cv::Scalar(max), channel_w[2]);
	channel_w[0]=cv::Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC1);//960 x 1280
	channel_w[2]=cv::Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC1);
	
	cv::merge(channel_w,3,img_w);
	cv::imshow("white",img_w);

 	cv::Mat img_b;
	cv::Mat chanB[3];
	cv::split(imgOriginal, chanB);
	cv::inRange(chanB[0], cv::Scalar(min), cv::Scalar(max), chanB[0]);
	//cv::inRange(channel_w[0], cv::Scalar(min), cv::Scalar(max), channel_w[0]);
	//cv::inRange(channel_w[2], cv::Scalar(min), cv::Scalar(max), channel_w[2]);
	chanB[1]=cv::Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC1);//960 x 1280
	chanB[2]=cv::Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC1);
	
	cv::merge(chanB,3,img_b);
	cv::imshow("blue",img_b);

        if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
       		cv::imwrite("red.png",img_red);
			cv::imwrite("bsaic.png",imgOriginal);
         std::cout << "esc key is pressed by user" << std::endl;
         break; 
       }
    }
	
cv::destroyAllWindows();	

   return 0;

}

