#include "projet.cpp"
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/*
 * 
 * 
 * *********************** PARTIE A MODIFIER ***********************
 * 
 * 
 * */
//lien vers les fichiers
const char* calibration_path = "calibration_matrix.txt";
const char* distorsion_path = "distorsion_matrix.txt";
std::string video_path = "/home/valentin/Bureau/projet/calibration5/videotest.mp4";

//affichage
bool affichage = true;

//intensité des LED
int min = 255;
int max = 255;
int val_min_red = 250;
int val_min_white = 250; 

//taille des LED
int rayon_zone_red = 45;
int rayon_zone_white = 30;
int seuil_nbPixel_red = 35 ;
int seuil_nbPixel_white = 20 ;

//géomètrie
double coeff_min_white = 0.8 ;
double coeff_max_white = 1.2 ;


int main(int argc, char** argv )
{
	//paramètre du modèle
	double cable = 1.5;
	std::vector<cv::Point3f> objPoints;
	objPoints.push_back(cv::Point3f(-0.237193325, -0.24218675, cable));
	objPoints.push_back(cv::Point3f(0.251283175, -0.23832675,cable));
	objPoints.push_back(cv::Point3f(-0.242993025, 0.23693425, cable));
	objPoints.push_back(cv::Point3f(0.228903175, 0.24357925, cable));
/*
 * 
 * 
 *********************** FIN DE LA PARTIE A MODIFIER ***********************
 * 
 * 
 * */
	projet(cable,
			min,
			max,
			rayon_zone_red,
			seuil_nbPixel_red,
			val_min_red,
			coeff_min_white,
			coeff_max_white,
			rayon_zone_white,
			seuil_nbPixel_white,
			val_min_white,
			affichage ,

			video_path,
			objPoints,
			calibration_path,
			distorsion_path
		);

}

