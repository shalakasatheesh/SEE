#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"

#include <iostream>

using namespace std;
using namespace cv;

int main(int argv , char** argc)
{

cout<<argv<<endl;

VideoCapture stream1(argv);
Mat frame1;
namedWindow("Display Image", WINDOW_AUTOSIZE );



if(!stream1.isOpened()){

        cout <<"Camera not opened" << endl;
    }
    else{
        while(true)
        {
        stream1.read(frame1);
        
        
        // printf("Camera capturing....\n");


        imshow("Display Image", frame1);
        waitKey(30);

        }
    }


}