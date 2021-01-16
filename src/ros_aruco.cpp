//
// Created by askat on 5/15/19.
//

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Transform.h>
#include "gazebo_msgs/SetModelState.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>
#include <iostream>
#include "ros_aruco.h"
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h>

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <stdlib.h>
#include <iostream>





#define SERVER_PORT htons(10105)
//#define CLIENT_PORT htons(10106)
using namespace std;

int ArUcoNode::arucoDetect() {
    
    ros::Rate r(10); // 10 hz
    ofstream file;
    file.open("Dataset.txt");
    file<<"ID Time X-axis Y-axis Z-axis Roll Pitch Yaw Closest_marker O_ID"<<endl;


	cout<<"Hello"<<endl;
        char buffer[1000];
        char buffer2[1000];
        int n;
        int n2;

        int serverSock=socket(AF_INET, SOCK_STREAM, 0);

        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = SERVER_PORT;
        serverAddr.sin_addr.s_addr = INADDR_ANY;


        bind(serverSock, (struct sockaddr*)&serverAddr, sizeof(struct sockaddr));
	    cout<<"Hello1"<<endl;
        // wait for a client
	    listen(serverSock,2);
        bzero(buffer, 1000);
        bzero(buffer2, 1000);

        sockaddr_in clientAddr;
        socklen_t sin_size=sizeof(struct sockaddr_in);
	    cout<<"Hello3"<<endl;
        int clientSock=accept(serverSock,(struct sockaddr*)&clientAddr, &sin_size);
        cout<<"Go go"<<endl;
////////////////////////////////////////        
        //int serverSock2=socket(AF_INET, SOCK_STREAM, 0);

        //sockaddr_in serverAddr2;
        //serverAddr2.sin_family = AF_INET;
        //serverAddr2.sin_port = CLIENT_PORT;
        //serverAddr2.sin_addr.s_addr = INADDR_ANY;


        //bind(serverSock2, (struct sockaddr*)&serverAddr2, sizeof(struct sockaddr));
	    //cout<<"Hello2"<<endl;

	    //  char hello = 'H'; 
	    //send(serverSock2, hello,strlen(hello),0);
        // wait for a client
	    //listen(serverSock2,2);
        //bzero(buffer, 1000);

        //sockaddr_in clientAddr2;
        //socklen_t sin_size2=sizeof(struct sockaddr_in);
	    //cout<<"Hello4"<<endl;
        //int clientSock2=accept(serverSock2,(struct sockaddr*)&clientAddr2, &sin_size2);
/////////////////////////////////////////
        int counter=0;
        while(ros::ok()){
             
             //receive a message from a client
             n = read(clientSock, buffer, 500);
             counter++;
             //send(clientSock , hello , strlen(hello),0);
             cout << "Confirmation code:  " << n << endl;
             cout << "Server received:  " << buffer << endl;
	         string s = buffer;
             string delimiter = ":";
	         string token[100];
	         size_t pos = 0;
   	         int i=0;
   	         int numOfMarkers=0;
		     if((pos=s.find(delimiter)!=string::npos)){
		        token[i] = s.substr(0, pos);
		        numOfMarkers=stoi(token[i]);
		        
		        s.erase(0, pos + delimiter.length());
	            i++;
		     }
		    cout<<"numOfMarkers:"<<numOfMarkers<<endl;
	        while ((pos = s.find(delimiter)) != string::npos){//&&(i<=(numOfMarkers*9+num))) {
	         token[i] = s.substr(0, pos);
	         s.erase(0, pos + delimiter.length());
	         i++;
	        }
	        token[i]=s;

	     int token2Length=numOfMarkers*9;
	     vector<int> ids(numOfMarkers);
	     vector<vector<cv::Point2f>> corners(numOfMarkers, vector<cv::Point2f>(4));
	     
	     string token2[token2Length];
	     for(int j=0;j<token2Length;j++){
	        token2[j]=token[j+1];
	     }
	     int numOfIds=stoi(token[token2Length+1]);
	     cout<<"numOfIds:"<<numOfIds<<endl;
	     string token3[numOfIds][5];
	     for(int j=0;j<numOfIds;j++){
	        for(int k=0;k<5;k++){
	            token3[j][k]=token[token2Length+j+k+2];
	            cout<<"token3:"<<token3[j][k]<<endl;
	        }
	     }

            //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
            //////////////////cap >> image; // get a new frame from camera
            //sharpen the frame
            //cv::GaussianBlur(frame, image, cv::Size(0, 0), 5);
            //cv::addWeighted(frame, 1.5, image, -0.5, 0, image);
            ////////////////std::vector<int> ids;
            ////////////////std::vector<std::vector<cv::Point2f>> corners;
            

            // if at least one marker detected
           
	    if(numOfMarkers!=0){
	        cout<<"Size of token:="<<sizeof(token2)/sizeof(token2[0])<<endl;
	        
	        for(int j=0;j<numOfMarkers;j++){
        	corners[j][0] = cv::Point2f(stof(token2[j*9+1]),stof(token2[j*9+2]));
        	corners[j][1] = cv::Point2f(stof(token2[j*9+3]),stof(token2[j*9+4]));
		    corners[j][2] = cv::Point2f(stof(token2[j*9+5]),stof(token2[j*9+6]));
		    corners[j][3] = cv::Point2f(stof(token2[j*9+7]),stof(token2[j*9+8]));
		    ids[j]=stoi(token2[j*9+0]);
    			
		    cout<<"ID:"<<ids[j]<<";Corners:";
		    for (int k = 0; k < 4; k++){ 
			    cout<< corners[j][k]<< ":"; 
		    } 
		    cout<< "\n"; 
		    }
                // for gazebo simulator
                gazebo_msgs::ModelState modelstate;

                geometry_msgs::PointStamped ps;
                ps.header.stamp = ros::Time::now();

                // draw detected markers
                // cv::aruco::drawDetectedMarkers(image, corners, ids);
                std::vector<cv::Vec3d> rvecs, tvecs;
                // estimate pose of each marker
                cv::aruco::estimatePoseSingleMarkers(corners, aruco_len, cameraMatrix, distCoeffs, rvecs, tvecs);
                Eigen::Vector3f pos_in_cf; // position in the camera frame
                Eigen::Vector4f pos_in_mf; // position in the marker frame
                Eigen::Vector4f pos_in_bf; // position in the base frameri
                
                Eigen::Matrix3f Rm; // rotation matrix: camera -> marker
                Eigen::Matrix4f Hb; // homogeneous matrix: marker -> base
                std::vector<Eigen::Vector4f> pos_vec;
                std::vector<tf2::Quaternion> q_bc;
                double area = 0.0;
                unsigned long ind = 0;
                // draw axis for each marker
                std::vector<tf2::Quaternion> q_mc;
                for(unsigned long i=0; i<ids.size(); ++i){

                    tf2::Quaternion q,q_new;
                    //double angle = norm(rvecs[i]);
                    //cv::Vec3d axis = rvecs[i] / angle;
                    //q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
                    q.setRPY(rvecs[i][0],rvecs[i][1],rvecs[i][2]);

                    // marker position in the camera frame
                    pos_in_cf << tvecs[i][0], tvecs[i][1], tvecs[i][2];
                    
                    // obtain the homogeneous matrix based on the marker id
                    Hb = transMap[ids[i]];
                    // obtain the rotation matrix: camera -> marker
                    Rm = rotMat(q.w(), q.x(), q.y(), q.z());
                  
                    // obtain the camera position in the marker frame
                    pos_in_mf << -Rm*pos_in_cf, 1;
                    // obtain the camera position in the base frame
                    pos_in_bf = Hb*pos_in_mf;
                    // push to the vector
                    pos_vec.push_back(pos_in_bf);

                    q_mc.push_back(q.inverse());
                    
                    q_new = q_mc[i]*q_bm[ids[i]];
                    
                    // Stuff the new rotation back into the pose. This requires conversion into a msg type
                    q_bc.push_back(q_new.normalize());

                    if(i==0){
                        area =  calcFiducialArea(corners[i]);
                        ind = i;
                    }else{
                        if(area < calcFiducialArea(corners[i])){
                            area =  calcFiducialArea(corners[i]);
                            ind = i;
                        }
                    }

                    //std::cout << "Marker ID: " << ids[i] << "; Fiducial marker area pxls: " << area << std::endl;
                    std::cout << "Marker ID: " << ids[i] << "; Marker position in the CF: " << pos_in_cf.transpose() << std::endl;
                    std::cout << "Marker ID: " << ids[i] << "; Camera position in the MF: " << pos_in_mf.transpose() << std::endl;
                    std::cout << "Marker ID: " << ids[i] << "; Camera position in the BF: " << pos_in_bf.transpose() << std::endl;

                    
                }
                    tf2::Quaternion q_orig, q_rot, q_new2;
   
                    // Get the original orientation of 'commanded_pose'
                    double r2=0,p2=3.14159,y2=0;
                    double r=0, p=0, y=0;  // Rotate the previous pose by 180* about X
                    q_rot.setRPY(r, p, y);
                    q_orig.setRPY(r2, p2, y2);
                    q_new2 = q_rot*q_orig;  // Calculate the new orientation
                    q_new2.normalize();
                    std::cout<<"Multiplication is ---"<<q_new2.x()<<"  " <<q_new2.y()<<"  "<<q_new2.z()<<"  "<<q_new2.w()<<std::endl;
                ps.point.x = pos_vec[ind](0);
                ps.point.y = pos_vec[ind](1);
                ps.point.z = pos_vec[ind](2);
		
                //std::cout << "The largest fiducial marker area pxls: " << area << std::endl;
                std::cout << "The closest marker ID: " << ids[ind] << "; Camera position in the BF: "  << ps.point.x << " " << ps.point.y << " " << ps.point.z << std::endl;
                std::cout << "Camera orientation in the BF: " << q_bc[ind].x() << " " << q_bc[ind].y() << " " << q_bc[ind].z() << " " << q_bc[ind].w() << std::endl;
 
                modelstate.model_name = (std::string) "unit_sphere_0";
                modelstate.pose.position.x = pos_vec[ind](0);
                modelstate.pose.position.y = pos_vec[ind](1);
                modelstate.pose.position.z = pos_vec[ind](2);
                
                modelstate.pose.orientation.x = q_bc[ind].x();
                modelstate.pose.orientation.y = q_bc[ind].y();
                modelstate.pose.orientation.z = q_bc[ind].z();
                modelstate.pose.orientation.w = q_bc[ind].w();
                EulerAngles angles=ToEulerAngles(q_bc[ind]);
                
                cout<<"WHYYYYY4"<<endl;
                file<<counter<<" "<<getTimestamp()<<" "<<to_string(pos_vec[ind](0))<<" "<<to_string(pos_vec[ind](1))<<" "<<to_string(pos_vec[ind](2))<<" "<<to_string(angles.roll)<<" "<<to_string(angles.pitch)<<" "<<to_string(angles.yaw)<<" "<<ids[ind]<<" ";
                
                gazebo_msgs::SetModelState srv;
                srv.request.model_state = modelstate;
                
                cout << "------------------" << std::endl;
                if(ps.point.y < y_max && ps.point.y > y_min){
                    pose_pub.publish(ps);
                    if(gazebo_client.call(srv)){
                        ROS_INFO("Successfully sent to gazebo!!");
                    }
                    else{
                        ROS_ERROR("Failed to sent to gazebo! Error msg:%s",srv.response.status_message.c_str());
                    }
                }

            }
        else{
            file<<counter<<" "<<getTimestamp()<<" -1 -1 -1 -1 -1 -1 -1 ";
        }
        cout<<"numOfIds:"<<numOfIds;
        file<<numOfIds<<":";
        if(numOfIds!=0){
            for(int l=0;l<numOfIds;l++){
                for(int m=0;m<5;m++){
                    file<<token3[l][m];
                    if(m!=4){
                        file<<",";
                    }
                    cout<<token3[l][m];
                }
                file<<";";
            }
           cout<<endl; 
        }
        
        
        //auto t1 = high_resolution_clock::now();
        
        file<<endl;
                
	    corners.clear();
	    ids.clear();
	    
        // cv::imshow("ArUcO Detection", image);
        cv::waitKey(1);
        r.sleep();
        /*std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
          std::cout<<1.0/time_span.count() <<std::endl;*/    
    }
    file.close();
}



double ArUcoNode::datenum(int year, int mon, int day, int hour, int imin, int sec, int mil){
	int tmp1, tmp2, tmp3;
	double	tmp4, tmp5;
	double dNum;
	static int cumdays[] = {0, 0,31,59,90,120,151,181,212,243,273,304,334};

	/* Calculate the serial date number:*/
	tmp1 = 365 * year  + cumdays[mon] + day;
	tmp2 = year / 4 - year / 100 + year / 400;
	tmp3 = (year % 4 != 0) - (year % 100 != 0) + (year % 400 != 0);
	tmp4 = (double) (tmp1+tmp2+tmp3);
	tmp5 = (hour * 3600000 + imin * 60000 + sec * 1000 + mil) / 86400000.0;

	dNum = tmp4 + tmp5;

	if (mon > 2) {
		if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)) {
			dNum += 1.0;
		}
	}

	return(dNum);
}
ArUcoNode::EulerAngles ArUcoNode::ToEulerAngles(tf2::Quaternion q){
    EulerAngles angles;
    
    //roll(x-axis direction)
    double sinr_cosp=2*(q.w()*q.x()+q.y()*q.z());
    double cosr_cosp=1-2*(q.x()*q.x()+q.y()*q.y());
    angles.roll=atan2(sinr_cosp,cosr_cosp);
    
    //pitch(y-axis direction)
    double sinp=2*(q.w()*q.y()-q.z()*q.x());
    if(abs(sinp)>=1)
        angles.pitch=copysign(M_PI/2,sinp);//use 90 degrees if out of range
    else
        angles.pitch=asin(sinp);
        
    //yaw(z-axis direction)
    double siny_cosp=2*(q.w()*q.z()+q.x()*q.y());
    double cosy_cosp=1-2*(q.y()*q.y()+q.z()*q.z());
    angles.yaw=atan2(siny_cosp, cosy_cosp);
    
    return angles;

}
std::string ArUcoNode::getTimestamp() {
  // get a precise timestamp as a string
  const auto now = std::chrono::system_clock::now();
  const auto nowAsTimeT = std::chrono::system_clock::to_time_t(now);
  tm local_tm = *localtime(&nowAsTimeT);
  int year = local_tm.tm_year + 1900 ;
  int month = local_tm.tm_mon + 1;
  int day = local_tm.tm_mday;
  int hour = local_tm.tm_hour+5;
  int min = local_tm.tm_min+30;
  int sec = local_tm.tm_sec;

  const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  int ms= stoi(to_string(nowMs.count()));
  double dnum=datenum(year, month, day, hour, min, sec, ms);
  //std::stringstream nowSs;
  /*nowSs
      << std::put_time(std::localtime(&nowAsTimeT), "%a %b %d %Y %T")
      << '.' << std::setfill('0') << std::setw(3) << nowMs.count();*/
  return to_string(dnum);
}
double ArUcoNode::dist(const cv::Point2f &p1, const cv::Point2f &p2) {
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

double ArUcoNode::calcFiducialArea(const std::vector<cv::Point2f> &pts) {
    const cv::Point2f &p0 = pts.at(0);
    const cv::Point2f &p1 = pts.at(1);
    const cv::Point2f &p2 = pts.at(2);
    const cv::Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

Eigen::Matrix4f ArUcoNode::trMat(Eigen::Matrix4f &Hm, float &x, float &y, float &z, float &rx, float &ry, float &rz) {
    float cos_t = std::cos(ry*deg2rad);
    float sin_t = std::sin(ry*deg2rad);

    Hm << cos_t, 0.0, sin_t, x,
           0.0, 1.0, 0.0, y,
           -sin_t, 0.0, cos_t, z,
           0, 0, 0, 1;

    return Hm;
}

tf2::Quaternion ArUcoNode::bmQuat(tf2::Quaternion &q2,float &rx, float &ry, float &rz){
    //tf2::Quaternion q2;
    cv::Vec3d rvecs;
    //rvecs<<rx,ry,rz;
    //double angle = norm(rvecs);
    if(ry==90 || ry==-90)
    ry = ry * 3.14159265359 / 180+1.57;

    else
    ry = ry * 3.14159265359 / 180-1.57;
    rx = rx * 3.14159265359 / 180-1.57;
    rz = rz * 3.14159265359 / 180;
    //cv::Vec3d axis = rvecs / angle;
    //q2.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
    q2.setRPY(rx,ry,rz);
    q2.normalize();
    return q2;
}

Eigen::Matrix3f ArUcoNode::rotMat(double qw, double qx, double qy, double qz) {
    // Auxiliary variables to avoid repeated arithmetic
    double qw2 = qw*qw, qx2 = qx*qx, qy2 = qy*qy, qz2 = qz*qz;
    double qxqy = qx*qy, qwqz = qw*qz, qwqy = qw*qy, qxqz = qx*qz, qyqz = qy*qz, qwqx = qw*qx;
    Eigen::Matrix3f rotM;
    rotM << qw2 + qx2 - qy2 - qz2, 2*(qxqy - qwqz), 2*(qwqy + qxqz),
            2*(qwqz + qxqy), qw2 - qx2 + qy2 - qz2, 2*(qyqz - qwqx),
            2*(qxqz - qwqy), 2*(qwqx + qyqz), qw2 - qx2 - qy2 + qz2;

    //cout << "Rotation Matrix "<<rotM<<endl;
    return rotM.transpose();

}

void ArUcoNode::initialization() {
    Eigen::Matrix4f H;
    tf2::Quaternion q2;
    int id;
    float x, y, z, rx, ry, rz;

    std::string line;
    std::ifstream file(fileName);

    if (file.is_open())
    {
        std::cout << "Initialization";
        while ( getline(file, line) )
        {
            std::istringstream ss(line);
            ss >> id >> x >> y >> z >> rx >> ry >> rz;

            transMap[id] = trMat(H, x, y, z, rx, ry, rz);
            q_bm[id]=bmQuat(q2, rx, ry, rz);

            //std::cout << transMap[id] << '\n';
            std::cout << "." ;

        }
        std::cout << '\n';
        file.close();
    }

    else std::cout << "Unable to open file";

}

ArUcoNode::ArUcoNode():n_h_("") {
    // set camera parameters
    cameraMatrix = (cv::Mat_<float>(3,3) << 666.52349183,   0,  441.04189665, 0,   659.37164362, 236, 29693346,0, 0,1);//1.07722650e+03, 0.00000000e+00, 4.16768656e+02, 0.00000000e+00, 1.07778246e+03, 1.92345914e+02, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00);//1.08440106e+03, 0.00000000e+00, 3.81902987e+02,0.00000000e+00, 1.08526972e+03, 1.89386463e+02,0.00000000e+00, 0.00000000e+00, 1.00000000e+00);
    //1378.438623, 0.000000, 924.854927, 0.000000, 1381.202474, 560.433664, 0.000000, 0.000000, 1.000000);

    distCoeffs = (cv::Mat_<float>(1,5) << -4.84508353e-02,  2.79630248e-01,  6.80014833e-03,  8.17392130e-04, -9.64730774e-01);//-4.10713475e-02,  1.24356039e+00, -4.95145175e-03,  5.52341740e-03, -1.12065158e+01);
  //-1.68022482e-01,  1.03217415e+00, -6.95113007e-03, -1.76731725e-03, -3.22528961e+00);
    //0.103206, -0.139285, 0.001162, -0.005657, 0.000000);
    width=1920, height=1080, fps=30;

    // set marker parameters
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    aruco_len = 0.14; axis_len = 0.2;

// set detector parameters
    parameters = new cv::aruco::DetectorParameters();
    n_h_.param<double>("adaptiveThreshConstant", parameters->adaptiveThreshConstant, 7);
    n_h_.param<int>("adaptiveThreshWinSizeMax", parameters->adaptiveThreshWinSizeMax, 23);
    n_h_.param<int>("adaptiveThreshWinSizeMin", parameters->adaptiveThreshWinSizeMin, 3);
    n_h_.param<int>("adaptiveThreshWinSizeStep", parameters->adaptiveThreshWinSizeStep, 10);
    n_h_.param<int>("cornerRefinementMaxIterations", parameters->cornerRefinementMaxIterations, 50);
    n_h_.param<double>("cornerRefinementMinAccuracy", parameters->cornerRefinementMinAccuracy, 0.001);
    n_h_.param<int>("cornerRefinementWinSize", parameters->cornerRefinementWinSize, 3);
#if OPENCV_MINOR_VERSION==2
    nh.param<bool>("doCornerRefinement",detectorParams->doCornerRefinement, true);
#else
    bool doCornerRefinement = true;
    n_h_.param<bool>("doCornerRefinement", doCornerRefinement, true);
    if (doCornerRefinement) {
        bool cornerRefinementSubPix = true;
        n_h_.param<bool>("cornerRefinementSubPix", cornerRefinementSubPix, true);
        if (cornerRefinementSubPix) {
            parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        }
        else {
            parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        }
    }
    else {
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }
#endif
    n_h_.param<double>("errorCorrectionRate", parameters->errorCorrectionRate , 0.6);
    n_h_.param<double>("minCornerDistanceRate", parameters->minCornerDistanceRate , 0.05);
    n_h_.param<int>("markerBorderBits", parameters->markerBorderBits, 1);
    n_h_.param<double>("maxErroneousBitsInBorderRate", parameters->maxErroneousBitsInBorderRate, 0.04);

    n_h_.param<int>("minDistanceToBorder", parameters->minDistanceToBorder, 1);
    n_h_.param<double>("minMarkerDistanceRate", parameters->minMarkerDistanceRate, 0.05);
    n_h_.param<double>("minMarkerPerimeterRate", parameters->minMarkerPerimeterRate, 0.04); // default 0.3
    n_h_.param<double>("maxMarkerPerimeterRate", parameters->maxMarkerPerimeterRate, 4.0);
    n_h_.param<double>("minOtsuStdDev", parameters->minOtsuStdDev, 5.0);
    n_h_.param<double>("perspectiveRemoveIgnoredMarginPerCell", parameters->perspectiveRemoveIgnoredMarginPerCell, 0.13);
    n_h_.param<int>("perspectiveRemovePixelPerCell", parameters->perspectiveRemovePixelPerCell, 4);
    n_h_.param<double>("polygonalApproxAccuracyRate", parameters->polygonalApproxAccuracyRate, 0.01);

    // ROS publisher parameters
    pose_pub = n_h_.advertise<geometry_msgs::PointStamped>("/camera_pose", 1);
    gazebo_client = n_h_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    // initialize transformation matrix
    initialization();
    // start pose estimation algorithm
    arucoDetect();
}
