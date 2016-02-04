#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Maths methods
#define max(a, b) ((a) > (b) ? (a) : (b)) 
#define min(a, b) ((a) < (b) ? (a) : (b)) 
#define abs(x) ((x) > 0 ? (x) : -(x)) 
#define sign(x) ((x) > 0 ? 1 : -1)

double euclideanDist(cv::Point2f & p, cv::Point2f & q) 
{
    cv::Point2f diff = p - q;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//Calculer le barycentre d'un vecteur de points

cv::Point2f calc_bary(std::vector<cv::Point2f> v)
{
	int bary_i = 0 ;
	int bary_j = 0 ;

	for(int k=0;k<v.size();k++)
	{
		bary_i += v[k].y;
		bary_j += v[k].x;
	}

	bary_i /= v.size();
	bary_j /= v.size();

	return cv::Point2f(bary_j,bary_i);
	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//Calculer la coordonnée i du point de la droite (aj+b)
void calc_point(double a, double b, int j, cv::Point2f & p)
{
	int i = a*j+b;
	p.y = i;
	p.x = j;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Calculer les coefficient a et b de la droite passant par (i1,j1) et (i2,j2)
void calc_droite(int i1,int j1,int i2,int j2,std::vector<double> & d)
{
	double a = (double)(i1-i2)/(double)(j1-j2);
	double b = i1-a*j1;
	d.push_back(a);
	d.push_back(b);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Calculer les coefficient a et b de la droite perpendiculaire a la droite passant par (i1,j1) et (i2,j2)
void calc_droite_perp(int i1,int j1,int i2,int j2,std::vector<double> & d)
{
	double a_perp = -(double)(j1-j2)/(double)(i1-i2);
	double b_perp = i1-a_perp*j1;
	d.push_back(a_perp);
	d.push_back(b_perp);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
// renvoit un vecteur contenant les coordonnées des points détectés dans l'image 
std::vector<cv::Point2f> find_red(cv::Mat W,int h, int w, int val_min,int rayon_zone,int seuil_nbPixel)
{
	//dilater les points
	cv::Mat I = W;
	//cv::Mat kernel = cv::getStructuringElement(2, cv::Size(5,5));
	//cv::erode(W, I, kernel);

	//rechercher les pixels rouges
	std::vector<cv::Point2f> v;//ensemble des pixels rouges d'une zone
	std::vector<cv::Point2f> reds;//ensemble des barycentres d'une zone de pixel rouge
		
	for(int j=0;j<w;j++)
	{
		for(int i=0;i<h;i++)
		{
			if(I.at<unsigned char>(i,j) > val_min) //pixel rouge détecté
			{
				//on regarde si il y a d'autres pixels autour
				if(i+rayon_zone < h && j+rayon_zone < w)//conditions au bords 
				{		
					int nbPixel = 0;
					for(int k=0;k<rayon_zone;k++)
					{
						if(I.at<unsigned char>(i+k,j) > val_min)
						{nbPixel++;}
						if(I.at<unsigned char>(i,j+k) > val_min)
						{nbPixel++;}
						if(I.at<unsigned char>(i+k,j+k) > val_min)
						{nbPixel++;}
					}
					//std::cout << "nbpixel rouge : " << nbPixel << std::endl;
					
					if(nbPixel >seuil_nbPixel) //point détecté !	
					{
						for(int k1=max(0,i-rayon_zone);k1<min(i+rayon_zone,h);k1++)
						{
							for(int k2=max(0,j-rayon_zone);k2<min(w,j+rayon_zone);k2++)
							{
								if(I.at<unsigned char>(k1,k2) > val_min)
								{
									I.at<unsigned char>(k1,k2) =0;
									cv::Point2f p(k2,k1);
									v.push_back(p);
								}
							}
						}
						cv::Point2f bary = calc_bary(v) ;
						reds.push_back(bary);
						v.clear();
					}
				}
			}
		}
	}
	
	return reds;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
// trouver les points blanc en fonction des rouges

void find_white(cv::Mat & Ig,int i,int j, double a, double b, int w, int h, int rayon_zone, int val_min, double dist_min_red, double dist_max_red, int seuil_nbPixel,std::vector<cv::Point2f> & p_whites)
{
	/*
		Ir,Ig,Ib = channels RGB
		i,j = point rouge de la droite
		a,b = parametres de la droite : i = aj+b
		w,h = taille de l'image
		r   = taille de la zone de recherche
	*/

	std::vector<cv::Point2f> v;
	cv::Point2f p1(j,i) ;
	cv::Point2f P(0,0);

	for(int jw = j-rayon_zone; jw<j+rayon_zone ; jw++)
	{
		//calculer le point de la droite

		cv::Point2f p;
		calc_point(a,b,jw,p);
		
		//verifier qu'il est dans l'image
		if(p.y >= 0 && p.y < h)
		{
			//Définir une zone de recherche			
			int i_min = max(0,p1.y+dist_min_red);
			int i_max = min(p1.y+dist_max_red,h);
			
			//tester les points de la zone
			for(int iw=i_min ; iw<i_max;iw++)
			{
				cv::Point2f P(jw,iw);	
				if((int)Ig.at<unsigned char>(iw,jw) > val_min)
				{	
					//on regarde si il y a d'autres pixels autour
					if(iw+rayon_zone < h && jw+rayon_zone < w)//conditions au bords 
					{		
						int nbPixel = 0;
						for(int k=0;k<rayon_zone;k++)
						{
							if(Ig.at<unsigned char>(iw+k,jw) > val_min)
							{nbPixel++;}
							if(Ig.at<unsigned char>(iw,jw+k) > val_min)
							{nbPixel++;}
							if(Ig.at<unsigned char>(iw+k,jw+k) > val_min)
							{nbPixel++;}
						}
						//std::cout << "nbpixel blanc : " << nbPixel << std::endl;
						if(nbPixel >seuil_nbPixel  ) //point détecté !	
						{
							for(int k1=max(0,iw-rayon_zone);k1<min(iw+rayon_zone,h);k1++)
							{
								for(int k2=max(0,jw-rayon_zone);k2<min(w,jw+rayon_zone);k2++)
								{
									if(Ig.at<unsigned char>(k1,k2) > val_min)
									{
										Ig.at<unsigned char>(k1,k2) =0;
										cv::Point2f p(k2,k1);
										v.push_back(p);

									}
								}
							}
							cv::Point2f bary = calc_bary(v) ;
							p_whites.push_back(bary);
							v.clear();
						}
					}
				}
			}
		}
	}
}



void find_p(cv::Mat * composante_imgOriginal, cv::Mat * composante_imgred, std::vector<cv::Point2f> & points, int rayon_zone_red, int seuil_nbPixel_red, int val_min_red, double coeff_min_white, double coeff_max_white,int rayon_zone_white, int seuil_nbPixel_white, int val_min_white)
{
	int w = composante_imgOriginal[2].cols;
	int h = composante_imgOriginal[2].rows;
	bool flag=true;
	
	///////////////// Chercher les points rouges //////////////////////
	points = find_red(composante_imgred[2],h,w,val_min_red,rayon_zone_red,seuil_nbPixel_red); // Liste des points rouges detéctés
	
	int nbPointsDetec = points.size();
	//std::cout << "points rouges déctectés : " << nbPointsDetec << std::endl;

	if(nbPointsDetec != 2)
	{
	//////////////// si les rouges ne sont pas trouvés, ou qu'il y a plus de 2 points détéctés //////////////////////////////
	std::cout << "points rouges déctectés : " << nbPointsDetec << std::endl;
	std::cout <<  "PAS ENCORE IMPLEMENTE" << std::endl;
	//flag=true;	
	}

	if(flag)
	{
	//////////////// les points rouges sont trouvés, chercher les droites pour definir une zone de recherche /////////////////////

		int i1 = points[0].y ; 
		int j1 = points[0].x ; 
		int i2 = points[1].y ; 
		int j2 = points[1].x ; 
	
		cv::inRange(composante_imgOriginal[1], cv::Scalar(255), cv::Scalar(255), composante_imgOriginal[1]);
		cv::Mat kernel = cv::getStructuringElement(2, cv::Size(5,5));
		cv::erode(composante_imgOriginal[1], composante_imgOriginal[1], kernel);
		cv::dilate(composante_imgOriginal[1], composante_imgOriginal[1], kernel);
		
		std::vector<double> d ;
		std::vector<double> dperp1;	
		std::vector<double> dperp2;
		calc_droite(i1,j1,i2,j2,d);
		calc_droite_perp(i1,j1,i2,j2,dperp1);
		calc_droite_perp(i2,j2,i1,j1,dperp2);			

		cv::Point2f p1;
		cv::Point2f p2;
		cv::Point2f pstart_perp1;
		cv::Point2f pend_perp1;
		cv::Point2f pstart_perp2;
		cv::Point2f pend_perp2;
		calc_point(d[0],d[1],j1,p1);
		calc_point(d[0],d[1],j2,p2);

		cv::Point2f pstart;
		cv::Point2f pend;
		calc_point(d[0], d[1], 0, pstart);
		calc_point(d[0], d[1], w, pend);
		
		calc_point(dperp1[0],dperp1[1],0,pstart_perp1);
		calc_point(dperp1[0],dperp1[1],w,pend_perp1);
		calc_point(dperp2[0],dperp2[1],0,pstart_perp2);
		calc_point(dperp2[0],dperp2[1],w,pend_perp2);
		/*
		cv::line(composante_imgOriginal[0], pstart, pend, cv::Scalar(255,255,255));
		cv::line(composante_imgOriginal[0], pstart_perp1, pend_perp1, cv::Scalar(255,255,255));
		cv::line(composante_imgOriginal[0], pstart_perp2, pend_perp2, cv::Scalar(255,255,255));
		
		cv::Point2f pdec1;
		cv::Point2f pdec2;
		cv::Point2f pdec3;
		cv::Point2f pdec4;
		
		pdec1.x = pstart_perp1.x - rayon_zone_white;
		pdec3.x = pend_perp1.x - rayon_zone_white;
		pdec2.x = pstart_perp1.x + rayon_zone_white;
		pdec4.x = pend_perp1.x + rayon_zone_white;
		pdec1.y = pstart_perp1.y;
		pdec3.y = pend_perp1.y;
		pdec2.y = pstart_perp1.y;
		pdec4.y = pend_perp1.y;
		cv::line(composante_imgOriginal[0], pdec1, pdec3, cv::Scalar(255,255,255));
		cv::line(composante_imgOriginal[0], pdec2, pdec4, cv::Scalar(255,255,255));
		
		cv::Point2f pdec12;
		cv::Point2f pdec22;
		cv::Point2f pdec32;
		cv::Point2f pdec42;
		
		pdec12.x = pstart_perp2.x - rayon_zone_white;
		pdec32.x = pend_perp2.x - rayon_zone_white;
		pdec22.x = pstart_perp2.x + rayon_zone_white;
		pdec42.x = pend_perp2.x + rayon_zone_white;
		pdec12.y = pstart_perp2.y;
		pdec32.y = pend_perp2.y;
		pdec22.y = pstart_perp2.y;
		pdec42.y = pend_perp2.y;
		cv::line(composante_imgOriginal[0], pdec12, pdec32, cv::Scalar(255,255,255));
		cv::line(composante_imgOriginal[0], pdec22, pdec42, cv::Scalar(255,255,255));
		*/
		//calculer la distance entre les leds rouges
		cv::Point2f  p01(j1,i1);
		cv::Point2f  p02(j2,i2);
		double dist_red = euclideanDist(p01,p02);
		double dist_min_red = coeff_min_white * dist_red;
		double dist_max_red = coeff_max_white * dist_red;
		
		//Rechercher les points blancs
		find_white(composante_imgOriginal[1],i1,j1, dperp1[0], dperp1[1], w, h, rayon_zone_white,val_min_white,dist_min_red,dist_max_red,seuil_nbPixel_white,points);
		find_white(composante_imgOriginal[1],i2,j2, dperp2[0], dperp2[1], w, h, rayon_zone_white,val_min_white,dist_min_red,dist_max_red,seuil_nbPixel_white,points);
	
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
double cable = 1.5;
std::vector<cv::Point3f> objPoints;
/*
228,903175	243,57925
-237,193325	-242,18675
-242,993025	236,93425
251,283175	-238,32675
*/
objPoints.push_back(cv::Point3f(-0.237193325, -0.24218675, cable));
objPoints.push_back(cv::Point3f(0.251283175, -0.23832675,cable));
objPoints.push_back(cv::Point3f(-0.242993025, 0.23693425, cable));
objPoints.push_back(cv::Point3f(0.228903175, 0.24357925, cable));

std::cout << "position des points 3D : " << std::endl << objPoints  << std::endl;

//points dans le repere camera
std::vector<cv::Point2f> imgPoints;
std::vector<cv::Point2f> imgPointsReproj;
	
//chargement des fichiers de calibration de la camera
cv::Mat cam = cv::Mat::zeros(3, 3, CV_64FC1);
cv::Mat distorsion = cv::Mat::zeros(1, 5, CV_64FC1);
const char* filename = "calibration_matrix.txt";
readFileToMat(cam,filename);  
filename = "distorsion_matrix.txt";
readFileToMat(distorsion,filename);

cv::VideoCapture cap("/home/valentin/Bureau/projet/calibration5/VIDEOTEST 2.mp4"); 

    if ( !cap.isOpened() ) 
    {
         std::cout << "Cannot open the web cam" << std::endl;
         return -1;
    }

   cv::Mat imgOriginal;
   cv::Mat imgFinal;
	int min = 255;
	int max = 255;
	int nbpts = 0;
	int seuil = 200;
	
	// param
	int rayon_zone_red = 40;
	int seuil_nbPixel_red = 20 ;
	int val_min_red = 250;
	double coeff_min_white = 0.6 ;
	double coeff_max_white = 1.0 ;
	int rayon_zone_white = 20;
	double seuil_nbPixel_white = 20 ;
	int val_min_white = 250; 
	// param

	cv::namedWindow("Original", CV_WINDOW_AUTOSIZE );	
	cv::moveWindow("Original",10,10);
	//cv::resizeWindow("Original", 400, 300);
	cv::namedWindow("final", CV_WINDOW_AUTOSIZE );	
	cv::moveWindow("final",440,10);
	//cv::resizeWindow("final", 400, 300);
	
//faire boucle sur video
 while (true)
    {
        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             std::cout << "Cannot read a frame from video stream" << std::endl;
             break;
        }
 	cv::imshow("Original", imgOriginal);

	cv::Mat channel_red[3];
 	cv::split(imgOriginal, channel_red);
 	
	//seuillage sur le rouge : on garde que les pixel entre min et max
	cv::inRange(channel_red[2], cv::Scalar(min), cv::Scalar(max), channel_red[2]);
	cv::Mat kernel = cv::getStructuringElement(2, cv::Size(5,5));
	cv::erode(channel_red[2], channel_red[2], kernel);
	cv::dilate(channel_red[2], channel_red[2], kernel);
	
	cv::Mat channel_white[3];
	cv::split(imgOriginal, channel_white);
	imgPoints.clear();
	/////////////////////////////////////////////////////////////////////////
	//ajout detection des pts et traitement

	find_p(channel_white, channel_red, imgPoints, rayon_zone_red, seuil_nbPixel_red, val_min_red, coeff_min_white, coeff_max_white, rayon_zone_white, seuil_nbPixel_white, val_min_white);
	int rayon_cercle = 10;
	//std::cout << "nb pts detecté : " << points.size() << std::endl;
	
	cv::Mat result = cv::Mat::zeros(3, 4, CV_64FC1);
	solvePNP (objPoints, imgPoints, cam, distorsion, result);
	//std::cout << "matrice : " << std::endl << result << std::endl;

	cv::Mat rvec = result(cv::Rect(0,0,3,3));
	cv::Mat tvec = result(cv::Rect(3,0,1,3));
	imgPointsReproj.clear();
	cv::projectPoints(objPoints, rvec,tvec, cam, distorsion,imgPointsReproj);

	imgFinal = imgOriginal;
	cv::Mat channel_imgFinal[3];
	cv::split(imgFinal,channel_imgFinal);
	
	//points rouges
	for(int x=0;x<imgPoints.size()/2;x++)
	{
		cv::circle(channel_imgFinal[0], imgPoints[x],rayon_cercle, cv::Scalar(0,0,0));
		cv::circle(channel_imgFinal[1], imgPoints[x],rayon_cercle, cv::Scalar(0,0,0));
		cv::circle(channel_imgFinal[2], imgPoints[x],rayon_cercle, cv::Scalar(255,255,255));
		
		cv::circle(channel_imgFinal[0], imgPointsReproj[x],rayon_cercle, cv::Scalar(255,255,255));
		cv::circle(channel_imgFinal[1], imgPointsReproj[x],rayon_cercle, cv::Scalar(0,0,0));
		cv::circle(channel_imgFinal[2], imgPointsReproj[x],rayon_cercle, cv::Scalar(0,0,0));
	}
	//points verts
	for(int x=imgPoints.size()/2;x<imgPoints.size();x++)
	{
		cv::circle(channel_imgFinal[0], imgPoints[x],rayon_cercle, cv::Scalar(0,0,0));
		cv::circle(channel_imgFinal[1], imgPoints[x],rayon_cercle, cv::Scalar(255,255,255));
		cv::circle(channel_imgFinal[2], imgPoints[x],rayon_cercle, cv::Scalar(0,0,0));		
	
		cv::circle(channel_imgFinal[0], imgPointsReproj[x],rayon_cercle, cv::Scalar(0,0,0));
		cv::circle(channel_imgFinal[1], imgPointsReproj[x],rayon_cercle, cv::Scalar(0,0,0));
		cv::circle(channel_imgFinal[2], imgPointsReproj[x],rayon_cercle, cv::Scalar(0,0,0));		
	}

	cv::merge(channel_imgFinal,3,imgFinal);
	cv::imshow("final",imgFinal);
	//std::cout << "pts : " << std::endl << points << std::endl;
	
	double erreur = 0;
	for(int i = 0;i<imgPoints.size();i++)
	{
		erreur = erreur + (imgPoints[i].x - imgPointsReproj[i].x)*(imgPoints[i].x - imgPointsReproj[i].x) + (imgPoints[i].y - imgPointsReproj[i].y)*(imgPoints[i].y - imgPointsReproj[i].y);
	}
	erreur = erreur/imgPoints.size();
	std::cout << "erreur : " << erreur << std::endl;

	//fin des coords des pts en pixel
	//attention a la mise en correspondace
	///////////////////////////////////////////////////////////////////////////
	

        if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            std::cout << "esc key is pressed by user" << std::endl;
            break; 
       }
    }
	
cv::destroyAllWindows();	


  return 0;
}
