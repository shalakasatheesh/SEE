#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>

#include "Eigen/Dense"


#include <iostream>


using namespace std;
using namespace cv;
using namespace Eigen;



//Global coordinates


int chessboard[2]={7,7};// Size of the chessboard
                        // I assumed our chessboard was 8x8
                        // But apparently not //Dimensions of the checker board 7,7



//Function to uses

// The function to stream live camera feed
void stream_camera(int argv);


int main(int argc , char** argv)
{


  //  To run the code , type
  //  "./Assignment_3_1" in termnial

    // Since we need to declare a 3D point of the image
    // We define a vector of a vector each element being able to store 3 floating points that relate images 
    // 2D point to a real world 3D point
    // Unlike arrays size of vecotors can increase dyanmically
   std::vector<std::vector<cv::Point3f>> objpoints;

   //Create a vector to save 2D point of the chess board image
   vector<vector<cv::Point2f>> imgpoints;

   //The algorithm define the corner of the chess board as the world origin
   // This mean the z=0 for the surface of the chess board

   //Defining the 3D coordinates 
   vector<cv::Point3f > objp;



   for (int i{0}; i<chessboard[1];i++)
   {
       for(int j{0} ; j<chessboard[0];j++)
       {

           //Push back is simmilar to heap_push. add a vector to the last position of the array
           objp.push_back(cv::Point3f(j,i,0));
       }
   }


    //Vector to store image file names
    vector<cv::String>images;

    // Path of the folder containing chessboard images
    std::string path = "/home/malika/Documents/Bonn Stuff/SEE/Report/SEE/data/Assignment_3_1/calib_images/*.jpg";

    //The path to all images for calibration
    cv::glob(path,images);

    //Cv mat to save original and grayscaled images
    cv::Mat original, gray;

    //Vector to store the coreners detected on the chess board 
    vector<cv::Point2f> corner_points;

    //Check if finding corners were successfull
    bool if_sucess;

    // Window to diplay corners
    namedWindow("Corner_Image", WINDOW_AUTOSIZE );
    //Loop though all image

    for(int i{0};i<images.size();i++)
    {
        //load up the image
        original=imread(images[i]);
        //convert to grayscale
        cvtColor(original,gray,CV_BGR2GRAY);

        //finding corners
        if_sucess=findChessboardCorners(gray,cv::Size(chessboard[0],chessboard[1]),corner_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        
        /*

        Various operation flags that can be zero or a combination of the following values:

        CALIB_CB_ADAPTIVE_THRESH :

        Use adaptive thresholding to convert the image to black and white, rather than a 
        fixed threshold level (computed from the average image brightness).


        CALIB_CB_NORMALIZE_IMAGE:
         Normalize the image gamma with equalizeHist() before applying fixed or adaptive thresholding.


        CALIB_CB_FILTER_QUADS:
         Use additional criteria (like contour area, perimeter, square-like shape) to filter out 
         false quads extracted at the contour retrieval stage.

        CALIB_CB_FAST_CHECK :
        Run a fast check on the image that looks for chessboard corners,
         and shortcut the call if none is found. This can drastically speed up the call in the
          degenerate condition when no chessboard is observed.


        */


       if(if_sucess)
       {

           //Set the termination criteria for fiding corners at subpixel level
           TermCriteria stop_critera(CV_TERMCRIT_EPS| CV_TERMCRIT_ITER,30,0.001);

            //Perform the subpixel level corner detection
           cornerSubPix(gray,corner_points,cv::Size(11,11),cv::Size(-1,-1),stop_critera);

           //drawt the found corerns on the original image matrix

           drawChessboardCorners(original,cv::Size(chessboard[0],chessboard[1]),corner_points,if_sucess);
           /*
           complete board was found or not. The return value of findChessboardCorners should be passed here. 
           The function draws individual chessboard corners detected either as red circles if the board was not found,
            or as colored corners connected with lines if the board was found.
            
            */

           // Add the inital known 3d word coorndiated(corner of chess board) in pair with the corner points found
           objpoints.push_back(objp);
           imgpoints.push_back(corner_points);
           imshow("Corner_Image",original);
           waitKey(20);
       }     


    }

    destroyAllWindows();

    Mat cameraMat,dist_coefficent,R,T;

    calibrateCamera(objpoints,imgpoints,cv::Size(gray.rows,gray.cols),cameraMat,dist_coefficent,R,T);

    std::cout << "cameraMatrix : \n" << cameraMat << std::endl;
    cout<<""<<endl;
    std::cout << "distCoeffs : \n" << dist_coefficent << std::endl;
    cout<<""<<endl;
    cout<<""<<endl;
    cout<<""<<endl;
    std::cout << "Rotation vector : \n" << R << std::endl;
    cout<<""<<endl;
    cout<<""<<endl;
    cout<<""<<endl;
    std::cout << "Translation vector : \n" << T << std::endl;





    // Test code for Eigen

    // MatrixXd m = MatrixXd::Random(3,3);
    // m=(m+MatrixXd::Constant(3,3,1.2))*50;
    // cout<<"m ="<<m<<endl;
    // char camera_stream;
    // cout<<"Enter name"<<endl;
    // cin>>camera_stream;
    // stream_camera(int(camera_stream));



    return 0;
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