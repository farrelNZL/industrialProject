#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

void solvePNP (std::vector<cv::Point3f> objPoints, std::vector<cv::Point2f> imgPoints, cv::Mat cam, cv::Mat distorsion, cv::Mat output)
{

cv::Mat rvec_init = cv::Mat::zeros(3, 1, CV_64FC1);
cv::Mat tvec_init = cv::Mat::zeros(3, 1, CV_64FC1);
cv::solvePnP(objPoints,imgPoints,cam,distorsion,rvec_init,tvec_init,CV_EPNP);

//init avec EPNP
cv::Mat rvec = rvec_init;
cv::Mat tvec = tvec_init;

cv::solvePnP(objPoints,imgPoints,cam,distorsion,rvec,tvec,CV_ITERATIVE);

cv::Mat rmat;
cv::Rodrigues(rvec,rmat);
hconcat(rmat,tvec,output);
}
void readFileToMat(cv::Mat & m, const char* filename)
{
    std::ifstream fin(filename);

    if(!fin)
    {
        std::cout<<"File Not Opened"<<std::endl;  return;
    }

    for(int i=0; i<m.rows; i++)
    {
        for(int j=0; j<m.cols; j++)
        {
            fin>>m.at<double>(i,j);
        }
    }

    fin.close();
}

int main(int argc, char** argv )
{
//points dans le repere scene ou monde (position en metre)
double position = 0.02375;
double cable = 1;
std::cout << "position : "  << position << " longueur : " << cable << std::endl;

std::vector<cv::Point3f> objPoints;
objPoints.push_back(cv::Point3f(-position,position,cable));
objPoints.push_back(cv::Point3f(position,position,cable));
objPoints.push_back(cv::Point3f(position,-position,cable));
objPoints.push_back(cv::Point3f(-position,-position,cable));

//chargement des fichiers de calibration de la camera
cv::Mat cam = cv::Mat::zeros(3, 3, CV_64FC1);
cv::Mat distorsion = cv::Mat::zeros(1, 5, CV_64FC1);
const char* filename = "calibration_matrix.txt";
readFileToMat(cam,filename);  
filename = "distorsion_matrix.txt";
readFileToMat(distorsion,filename);

//faire boucle sur video
cv::VideoCapture cap("/home/valentin/Bureau/projet/GOPR0313.MP4"); 

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
	
 while (true)
    {
        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             std::cout << "Cannot read a frame from video stream" << std::endl;
             break;
        }
 
	 cv::Mat img_red;
    cv::Mat channel_red[3];
 
 	cv::split(imgOriginal, channel_red);
 	//les pts blancs sont supprimer pour reduire le "bruit" du ciel
	suppr_pts_blanc(channel_red, seuil);
		
	//seuillage sur le rouge : on garde que les pixel entre min et max
	cv::inRange(channel_red[2], cv::Scalar(min), cv::Scalar(max), channel_red[2]);
	//mise a 0 de la composante que l'on veut
	channel_red[0]=cv::Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC1);//960 x 1280
	channel_red[1]=cv::Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC1);
	
	//points dans le repere camera
	std::vector<cv::Point2f> imgPoints;

	/////////////////////////////////////////////////////////////////////////
	//ajout detection des pts et traitement
	nbpts = 0;
	
	//ici on fera le "barycentre" pour recuperer les coords des pts
	//puis les ajouter a imgPoints dans l'ordre
	for (int i = 0; i < imgOriginal.rows;i++)
	{
		for (int j = 0; j < imgOriginal.cols ; j++)
		{
			unsigned int val_r = channel_red[2].at<unsigned char>(i,j);
			if (	val_r > 0)
			{
				nbpts++;
				//std::cout << "pix " << j << ", " << i << " = " << val_r << std::endl;
			}
		}
	}
	
	//fin des coords des pts en pixel
	//attention a la mise en correspondace
	///////////////////////////////////////////////////////////////////////////
	
	cv::Mat result = cv::Mat::zeros(3, 4, CV_64FC1);  

	solvePNP (objPoints, imgPoints, cam, distorsion, result);
	std::cout << "matrice : " << std::endl << result << std::endl;


        if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            std::cout << "esc key is pressed by user" << std::endl;
            break; 
       }
    }
	
cv::destroyAllWindows();	


  return 0;
}
