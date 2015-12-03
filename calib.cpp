 //http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

//g++ -lopencv_core -lopencv_calib3d -lopencv_highgui calib.cpp


/*
double calibrateCamera(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints, Size imageSize, InputOutputArray cameraMatrix, InputOutputArray distCoeffs, OutputArrayOfArrays rvecs, OutputArrayOfArr  ys tvecs, int flags=0, TermCriteria criteria=TermCriteria( TermCriteria::COUNT+TermCriteria::EPS, 30, DBL_EPSILON) )

objectPoints -> Vecteurs de coordonnées des points de la mire dans le repère de la mire
					  (nbVecteurs = nombre de photos de la mire)

imagePoints -> Vecteurs de coordonnées des points de la mire projetés dans l'image

imageSize -> taille de l'image

cameraMatrix -> matrice des paramètres intrinseque

						|fx 0 u0|
						|0 fy v0|
						|0  0  1|
/!\/!\/!\ voir doc pour l'initialisation /!\/!\/!\

 
distCoeffs -> vecteur des coefficients de distorsion
				-> coefficients de distorsion ??

rvecs -> vecteurs de rotation correspondants à la position de la mire dans chaque photo
			(pour convertir une matrice de rotation en vecteur utilisable ici, voir doc Rodrigues)
tvecs -> idem pour les translations  


*/



/*

bool findChessboardCorners(InputArray image, Size patternSize, OutputArray corners, int flags=CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE )

image -> image de la mire
patternSize -> nombre de coins interieurs
				-> patternSize = cvSize(points_per_row,points_per_colum)
corners -> tableau de sortie des coins
			-> dans le sens : parcours ligne par ligne de la gauche vers la droite


Note

The function requires white space (like a square-thick border, the wider the better) around the board to make the detection more robust in various environments. Otherwise, if there is no border and the background is dark, the outer black squares cannot be segmented properly and so the square grouping and ordering algorithm fails.

*/
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
	std::string folder_path = "/home/oceane/Bureau/Tellus/img_calib/";
/*
	std::cout << "dossier contenant les images ?  --/home/oceane/Bureau/Tellus/img_calib/"" << std::endl ;
	std::cin >>  folder_path ;
*/
	int nbImage = 6 ;
	std::cout << "Nombre d'images ?  --6" << std::endl ;
	std::cin >>  nbImage ;

	std::string image_pref = "tellus";
	std::cout << "préfixe de l'image ? (nom de l'image du type prefixe-00.jpg)" << std::endl ;
	std::cin >>  image_pref ;
	image_pref = image_pref + "-";

	std::string image_ext;
	std::cout << "Extension des images ?  (.jpg,.png... sans point avant) --jpg" << std::endl ;
	std::cin >>  image_ext ;
	image_ext = "." + image_ext;


	int points_per_row = 9;
	int points_per_colum = 6;
	std::cout << "nombre de coins par lignes ? (uniquement les coins interieur de la mire)  --9" << std::endl ;
	std::cin >>  points_per_row ;
	std::cout << "nombre de coins par colonne ? (uniquement les coins interieur de la mire)  --6" << std::endl ;
	std::cin >>  points_per_colum ;

	double square_world_width = 0.029; //en metre
	std::cout << "taille d'un carré ? (en metres)  --0.029" << std::endl ;
	std::cin >>  square_world_width ;

	std::cout << " " << std::endl;
	std::cout << "------------------------------------------------------------------------" << std::endl;
	std::cout << "images de la forme : " << folder_path << image_pref << "00" << image_ext << std::endl;
	std::cout << "contenu : " << std::endl;
	std::cout << "-"<< points_per_row << " coins par lignes"  << std::endl;
	std::cout << "-"<< points_per_colum << " coins par colonnes"  << std::endl;
	std::cout << "taille des carrés = "<< square_world_width  << " m"<< std::endl;
	std::cout << "------------------------------------------------------------------------" << std::endl;
	std::cout << " " << std::endl;

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
    std::ofstream fout(filename);

    if(!fout)
    {
        std::cout<<"File Not Opened"<<std::endl;  return;
    }

    for(int i=0; i<m.rows; i++)
    {
        for(int j=0; j<m.cols; j++)
        {
            fout>>m.at<double>(i,j);
        }
    }

    fout.close();
}

int main( int argc, const char* argv[] )
{ 
	cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1); 
	cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);

	calibrer(cameraMatrix, distCoeffs);

	//Sauvegarde de la matrice
	const char* filename = "calibration_matrix.txt";
   writeMatToFile(cameraMatrix,filename);

	std::cout << "param " << std::endl << cameraMatrix << std::endl;
	std::cout << "distor " << std::endl << distCoeffs << std::endl;
	

	cv::Mat m;
	readFileToMat(m,filename)
	std::cout << "read mat " << std::endl << m<< std::endl;
	

	return 0; 
}
















