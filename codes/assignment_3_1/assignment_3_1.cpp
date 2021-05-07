#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"


#include <iostream>


using namespace std;
using namespace cv;


// The function to stream live camera feed
void stream_camera(int argv);
void load_images();

int main(int argc , char** argv)
{

    char camera_stream;
    cout<<"Enter name"<<endl;
    cin>>camera_stream;

    stream_camera(int(camera_stream));



    return 0;
}






void load_images()
{
    Mat images;




}




void stream_camera(int c)
{
cout<<"The cout is "<<c-48<<endl;

VideoCapture stream1(0);
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