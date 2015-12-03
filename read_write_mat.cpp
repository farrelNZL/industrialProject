#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//using namespace std;

void writeMatToFile(cv::Mat& m, const char* filename)
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
            fout<<m.at<float>(i,j)<<"\t";
        }
        fout<<std::endl;
    }

    fout.close();
}

int main()
{
    cv::Mat m = cv::Mat::eye(5,5,CV_32FC1);

    const char* filename = "output.txt";

    writeMatToFile(m,filename);

}
