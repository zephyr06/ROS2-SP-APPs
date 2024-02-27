/**
* This file is a modified version of ORB-SLAM2.<https://github.com/raulmur/ORB_SLAM2>
*
* This file is part of DynaSLAM.
* Copyright (C) 2018 Berta Bescos <bbescos at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/bertabescos/DynaSLAM>.
*
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<opencv2/core/core.hpp>
#include<memory>

#include "Geometry.h"
#include "MaskNet.h"
#include <System.h>
#include "yolo.h"

#define COMPILEDWITHC11

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

class DynaSLAMWrapperForROS2 {
public:
    DynaSLAMWrapperForROS2() {};
    // "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association (path_to_masks) (path_to_output)"

    void init() {
        vstrImageFilenamesRGB.clear();
        vstrImageFilenamesD.clear();
        vTimestamps.clear();

        // Retrieve paths to images
        strAssociationFilename = "/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM/Examples/RGB-D/associations/fr3_walking_xyz.txt";
        LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);// Check consistency in the number of images and depthmaps
        
        nImages = vstrImageFilenamesRGB.size();
        std::cout << "nImages: " << nImages << std::endl;
        nImages = 800;
        if(vstrImageFilenamesRGB.empty())
        {
            cerr << endl << "No images found in provided path." << endl;
            return;
        }
        else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
        {
            cerr << endl << "Different number of images for rgb and depth." << endl;
            return;
        }

        vTimesTrack.resize(nImages);


        string strVocFile = "/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM/Vocabulary/ORBvoc.txt";
        string strSettingsFile = "/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM/Examples/RGB-D/TUM3.yaml";
        p_SLAM = make_shared<ORB_SLAM2::System>(strVocFile, strSettingsFile, ORB_SLAM2::System::RGBD,true);

        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl << endl;


        // // Dilation settings
        // int dilation_size = 15;
        // cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
        //                                     cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
        //                                     cv::Point( dilation_size, dilation_size ) );

        image_idx = 0;

    }
    void next() {
        if (image_idx > 800) {
            image_idx = 0;
        }
        cout << image_idx << endl;
        // Read image and depthmap from file
        imRGB = cv::imread(string("/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/dataset/rgbd_dataset_freiburg3_walking_xyz/")+"/"+vstrImageFilenamesRGB[image_idx],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string("/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/dataset/rgbd_dataset_freiburg3_walking_xyz/")+"/"+vstrImageFilenamesD[image_idx],CV_LOAD_IMAGE_UNCHANGED);

        double tframe = vTimestamps[image_idx];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string("/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/dataset/rgbd_dataset_freiburg3_walking_xyz/") << "/" << vstrImageFilenamesRGB[image_idx] << endl;
            return;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Segment out the images
        cv::Mat mask = cv::Mat::ones(480,640,CV_8U);
        /*
        if (argc == 6 || argc == 7)
        {
            cv::Mat maskRCNN;
            maskRCNN = MaskNet->GetSegmentation(imRGB,string(argv[5]),vstrImageFilenamesRGB[image_idx].replace(0,4,""));
            cv::Mat maskRCNNdil = maskRCNN.clone();
            cv::dilate(maskRCNN,maskRCNNdil, kernel);
            mask = mask - maskRCNNdil;
        }
        */
        if (argc == 6 || argc == 7)
            mask = yolo->Segmentation(imRGB);
        // Pass the image to the SLAM system
        if (argc == 7){p_SLAM->TrackRGBD(imRGB,imD,mask,tframe,imRGBOut,imDOut,maskOut);}
        else {p_SLAM->TrackRGBD(imRGB,imD,mask,tframe);}

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        if (argc == 7)
        {
            cv::imwrite(string(argv[6]) + "/rgb/" + vstrImageFilenamesRGB[image_idx],imRGBOut);
            vstrImageFilenamesD[image_idx].replace(0,6,"");
            cv::imwrite(string(argv[6]) + "/depth/" + vstrImageFilenamesD[image_idx],imDOut);
            cv::imwrite(string(argv[6]) + "/mask/" + vstrImageFilenamesRGB[image_idx],maskOut);
        }

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[image_idx]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(image_idx<nImages-1)
            T = vTimestamps[image_idx+1]-tframe;
        else if(image_idx>0)
            T = tframe-vTimestamps[image_idx-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);

        image_idx++;
    }

    void exit() {
        // Stop all threads
        p_SLAM->Shutdown();

        // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        for(int ni=0; ni<nImages; ni++)
        {
            totaltime+=vTimesTrack[ni];
        }
        cout << "-------" << endl << endl;
        cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
        cout << "mean tracking time: " << totaltime/nImages << endl;

        // Save camera trajectory
        p_SLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
        p_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

        return;
    }
    
    int argc = 5;
    char **argv;

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename;
    int nImages;
    yolov3::yolov3Segment* yolo;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // string strVocFile = "/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM/Vocabulary/ORBvoc.txt";
    // string strSettingsFile = "/home/nvidia/workspace/sdcard/SP_Scheduler_Stack/YOLO-DynaSLAM/Examples/RGB-D/TUM3.yaml";
    // ORB_SLAM2::System SLAM(strVocFile, strSettingsFile, ORB_SLAM2::System::RGBD,true);
    shared_ptr<ORB_SLAM2::System> p_SLAM;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    // Main loop
    cv::Mat imRGB, imD;
    cv::Mat imRGBOut, imDOut,maskOut;
    
    int image_idx;
};
