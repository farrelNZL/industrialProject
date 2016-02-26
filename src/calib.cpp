 //http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html


#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
void calibrer(cv::Mat cameraMatrix, cv::Mat distCoeffs)
{

	bool flag = true; //while true, continue running

	////////////////// Chargement des paramètres /////////////////////////////////////////////

	// /!\ /!\ /!\ Attention, les images doivent être nommées sous la forme 'nom-00' avec 01,02,03... /!\ /!\ /!\ //


	cv::Size imageSize;
	std::string folder_path ;
	std::cout << "dossier contenant les images ?  exemple : /home/valentin/Bureau/projet/calibration4/" << std::endl ;
	std::cin >>  folder_path ;

	int nbImage = 6 ;
	std::cout << "Nombre d'images ?  exemple : 6" << std::endl ;
	std::cin >>  nbImage ;

	std::string image_pref = "tellus";
	std::cout << "préfixe de l'image ? (nom de l'image du type prefixe-00.jpg)" << std::endl ;
	std::cin >>  image_pref ;
	image_pref = image_pref + "-";

	std::string image_ext;
	std::cout << "Extension des images ?  (.jpg,.png... sans point avant)" << std::endl ;
	std::cin >>  image_ext ;
	image_ext = "." + image_ext;


	int points_per_row;
	int points_per_colum;
	std::cout << "nombre de coins par lignes ? (uniquement les coins interieur de la mire)  si mire fournie par opencv : 9" << std::endl ;
	std::cin >>  points_per_row ;
	std::cout << "nombre de coins par colonne ? (uniquement les coins interieur de la mire)  si mire fournie par opencv : 6" << std::endl ;
	std::cin >>  points_per_colum ;

	double square_world_width = 0.029; //en metre
	std::cout << "taille d'un carré sur la mire imprimée ? (en metres) " << std::endl ;
	std::cin >>  square_world_width ;

	std::cout << "------------------------------------------------------------------------" << std::endl;
	std::cout << "images de la forme : " << folder_path << image_pref << "00" << image_ext << std::endl;
	std::cout << "contenu : " << std::endl;
	std::cout << "-"<< points_per_row << " coins par lignes"  << std::endl;
	std::cout << "-"<< points_per_colum << " coins par colonnes"  << std::endl;
	std::cout << "taille des carrés = "<< square_world_width  << "m"<< std::endl;
	std::cout << "------------------------------------------------------------------------" << std::endl;

	//Création de la liste (des noms) des images
	std::vector<std::string> liste_image;
	for(int i=0;i<nbImage;i++)
	{
		std::string image_name = image_pref;

		if(i<10)
		{
			char nb[1] ;
			sprintf(nb, "%d", 0);
			image_name.append(nb);
		}

		char nb[1] ;
		sprintf(nb, "%d", i);
		image_name.append(nb);
		image_name.append(image_ext);
		liste_image.push_back(image_name);
	}


	std::vector<std::vector<cv::Vec3f> > objectPoints;
	std::vector<std::vector<cv::Vec2f> > imagePoints;
	

	for(int image=0;image<nbImage;image++)
	{
		
		std::string image_name = liste_image[image];
		std::string img_path = folder_path+image_name;

		//importer une image
		cv::Mat mire = cv::imread(img_path);
		if(mire.empty())
		{
		  std::cerr<<"Il y a un problème dans la calibration de l'image, veuillez vérifier que le dossier est situé au même endroit que le binaire\n ou que vous ne vous êtes pas trompés dans les paramètres précédents\n";
		 return ; 
		}
		imageSize = cv::Size(mire.cols,mire.rows);
		//déterminer les coordonnées 3D des points (dans le repère de la mire)
		std::vector<cv::Point3f> v ;
		for(int i=1;i<points_per_colum+1;i++)
		{
			for(int j=1;j<points_per_row+1;j++)
			{
				cv::Point3f pt(i*square_world_width,j*square_world_width,0);
				v.push_back(pt);
			}
		}
		objectPoints.push_back(cv::Mat(v));
		
		//extraire les points dans l'image
		std::vector<cv::Point2f> corners;
		cv::Size patternSize(points_per_row,points_per_colum); 
	 	bool corners_found = cv::findChessboardCorners(mire, patternSize, corners);
		if(!corners_found)
		{
			std::cout<<"erreur lors de l'extraction des coins de l'image"<<std::endl;
			std::cout<<"Verifiez les paramètres (nombres de coins par image)" << std::endl;
			std::cout<< "Si les paramètres sont justes, au moins l'une des images n'est pas utilisable"<<std::endl;
			flag = false;
		}
		imagePoints.push_back(cv::Mat(corners));
		if(!flag) {break;}
	}

	if(flag)
	{
		//calibrer la caméra
		std::vector<cv::Mat> rvecs ;
		std::vector<cv::Mat> tvecs ;
		
		cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs,rvecs, tvecs);
		//std::cout << "distor " << distCoeffs << std::endl;
	
	}

}

void writeMatToFile(cv::Mat & m, const char* filename)
{
    std::ofstream fout(filename);

    if(!fout)
    {
        std::cout<<"File Not Opened"<<std::endl;  return;
    }

    for(int i=0; i<m.rows; i++)
    {
        for(int j=0; j<m.cols; j++)
        {
            fout<<m.at<double>(i,j)<<"\t";
        }
        fout<<std::endl;
    }

    fout.close();
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

int main( int argc, const char* argv[] )
{ 
	cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1); 
	
	cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);
//Output vector of distortion coefficients (k1,k2,p1,p2[,k3[,k4,k5,k6],[s1,s2,s3,s4]]) of 4, 5, 8 or 12 elements.

	calibrer(cameraMatrix, distCoeffs);
	//Sauvegarde de la matrice
	const char* filename = "calibration_matrix.txt";
   writeMatToFile(cameraMatrix,filename);
	filename = "distorsion_matrix.txt";
	writeMatToFile(distCoeffs,filename);
	
	std::cout << "param " << std::endl << cameraMatrix << std::endl;
	std::cout << "distor " << std::endl << distCoeffs << std::endl;
	

	cv::Mat m = cv::Mat::zeros(3, 3, CV_64FC1);
	filename = "calibration_matrix.txt";
	readFileToMat(m,filename);
	std::cout << "read mat(calib matrix) " << std::endl << m<< std::endl;
	return 0; 
}












