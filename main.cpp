#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <hdf5_hl.h>


using namespace cv;
using namespace std;

int num_board=0;
int board_width =9, board_height=7;
int wait_time=0;
int time_sec=0;
Size board_size = Size(9,7);

bool cam= false;

int main ()
{

    VideoCapture cap(0);
    if(!cap.isOpened())  // check camera
        return -1;


    cout << "Enter no of board images :";
    cin >> num_board;
    cout << "Enter time in sec to take snaps: ";
    cin >> time_sec;

    int board_total  = board_width * board_height;
    //declare
    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points;
    vector<Point2f>corners;
    int sucess=0,frame=0,step=0;
    Mat image,gray_image;
    vector<Point3f> obj;
    for(int j=0;j<board_total;j++)
        obj.emplace_back(j/board_width, j%board_width, 0.0f);

    while (num_board >0)
    {
        namedWindow("image",WINDOW_NORMAL);
        cap.read(image);
//        else
//        image=imread("/home/nagesh/callibration/snaps/"+to_string(num_board)+".jpg");
        cvtColor(image,gray_image,COLOR_RGB2GRAY);
        if (image.empty())
        {
            printf("imahe is empty ");
            continue;
        }
        if((frame++ % time_sec) == 0)
        {
            bool f_chess= findChessboardCorners(image,board_size,corners,CV_CALIB_CB_ADAPTIVE_THRESH);
            cout<< f_chess<<endl;
            if(f_chess)
            {
                cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                drawChessboardCorners(gray_image, board_size, corners, f_chess);
                imshow("gray",gray_image);
                image_points.push_back(corners);
                object_points.push_back(obj);
                num_board--;
//                imwrite("/home/names/callibration/snaps/"+to_string(num_board)+".jpg",image);


            }

        }

        imshow("image", image);


        waitKey(100);



    }
    // image points and corner points
    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;
    calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
    Mat imageUndistorted;
    //save camera parameters in xml
    FileStorage fs;
    fs.open("Intrinsics.xml", FileStorage::WRITE);
    fs<<"Intrinsics"<<intrinsic;
    fs.release();
    fs.open("Distortion.xml", FileStorage::WRITE);
    fs<<"Distortion"<<distCoeffs;
    fs.release();
    printf("Files saved.\n\n");
    while(1)
    {
        cap >> image;
        undistort(image, imageUndistorted, intrinsic, distCoeffs);

        imshow("win1", image);
        imshow("win2", imageUndistorted);
        imshow("check distot",image-imageUndistorted);
        waitKey(1);
    }



    return 0;
}
