/***********************************************************************************
Name:           chessboard_pcd.cpp
Revision:
Date:           07/11/2018
Author:         Pedro Silva, Tiago Madeira
Comments:       ChessBoard Tracking


images
Revision:
Libraries:
Notes:          Code generated with Visual Studio 2013 Intel OpenCV 3.3.0 libraries 
***********************************************************************************/
#include <iostream>
#include <vector>

#include <stdio.h>

// OpenCV Includes
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL Includes
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/recognition/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>

// Name Spaces
using namespace cv;
using namespace std;
using namespace pcl;

int main(int argc, char **argv)
{
     /* Read Point Cloud files */
    PointCloud<PointXYZRGB>::Ptr cloud_0 (new PointCloud<PointXYZRGB>);
    if(io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor0.ply", *cloud_0) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor0.ply \n");
      return (-1);
    }

    PointCloud<PointXYZRGB>::Ptr cloud_1 (new PointCloud<PointXYZRGB>);
    if (io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor1.ply", *cloud_1) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor1.ply \n");
      return (-1);
    }

    PointCloud<PointXYZRGB>::Ptr cloud_2 (new PointCloud<PointXYZRGB>);
    if(io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor2.ply", *cloud_2) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor2.ply \n");
      return (-1);
    }
    
    PointCloud<PointXYZRGB>::Ptr cloud_3 (new PointCloud<PointXYZRGB>);
    if (io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor3.ply", *cloud_3) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor3.ply \n");
      return (-1);
    }
    
    PointCloud<PointXYZRGB>::Ptr cloud_4 (new PointCloud<PointXYZRGB>);
    if(io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor4.ply", *cloud_4) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor4.ply \n");
      return (-1);
    }
    
    PointCloud<PointXYZRGB>::Ptr cloud_5 (new PointCloud<PointXYZRGB>);
    if (io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor5.ply", *cloud_5) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor5.ply \n");
      return (-1);
    }
    
    PointCloud<PointXYZRGB>::Ptr cloud_6 (new PointCloud<PointXYZRGB>);
    if(io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor6.ply", *cloud_6) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor6.ply \n");
      return (-1);
    }
    
    PointCloud<PointXYZRGB>::Ptr cloud_7 (new PointCloud<PointXYZRGB>);
    if (io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor7.ply", *cloud_7) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor7.ply \n");
      return (-1);
    }
    
    PointCloud<PointXYZRGB>::Ptr cloud_8 (new PointCloud<PointXYZRGB>);
    if(io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor8.ply", *cloud_8) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor8.ply \n");
      return (-1);
    }
    
    PointCloud<PointXYZRGB>::Ptr cloud_9 (new PointCloud<PointXYZRGB>);
    if(io::loadPLYFile<PointXYZRGB> ("..//Depth_Images//pixelcolor9.ply", *cloud_9) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file pixelcolor9.ply \n");
      return (-1);
    }
    

    //Create Aligned Clouds and ICP point
    // PointCloud<PointXYZRGB>::Ptr align_cloud (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr align_cloud_a (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr align_cloud_b (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr align_cloud_c (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr align_cloud_d (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr align_cloud_e (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr align_cloud_f (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr align_cloud_g (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr align_cloud_h (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr align_cloud_i (new PointCloud<PointXYZRGB>);

    IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp_point;
    //set point transformations
    icp_point.setTransformationEpsilon (1e-6);
    icp_point.setMaxCorrespondenceDistance (0.25);
    icp_point.setMaximumIterations (50);
    
    //Align cloud 0 & 9 and save result in align_cloud_a
    icp_point.setInputCloud(cloud_8);
    icp_point.setInputTarget(cloud_9);
    icp_point.align (*align_cloud_a);
    //Align cloud 1 & 9 and save result in align_cloud_b
    icp_point.setInputCloud(cloud_7);
    icp_point.setInputTarget(align_cloud_a);
    icp_point.align (*align_cloud_b);
    //Align cloud 2 & 9 and save result in align_cloud_c
    icp_point.setInputCloud(cloud_6);
    icp_point.setInputTarget(align_cloud_b);
    icp_point.align (*align_cloud_c);
    //Align cloud 3 & 9 and save result in align_cloud_d
    icp_point.setInputCloud(cloud_5);
    icp_point.setInputTarget(align_cloud_c);
    icp_point.align (*align_cloud_d);
    //Align cloud 4 & 9 and save result in align_cloud_e
    icp_point.setInputCloud(cloud_4);
    icp_point.setInputTarget(align_cloud_d);
    icp_point.align (*align_cloud_e);
    //Align cloud 5 & 9 and save result in align_cloud_f
    icp_point.setInputCloud(cloud_3);
    icp_point.setInputTarget(align_cloud_e);
    icp_point.align (*align_cloud_f);
    //Align cloud 6 & 9 and save result in align_cloud_g
    icp_point.setInputCloud(cloud_2);
    icp_point.setInputTarget(align_cloud_f);
    icp_point.align (*align_cloud_g);
    //Align cloud 7 & 9 and save result in align_cloud_h
    icp_point.setInputCloud(cloud_1);
    icp_point.setInputTarget(align_cloud_g);
    icp_point.align (*align_cloud_h);
    //Align cloud 8 & 9 and save result in align_cloud_i
    icp_point.setInputCloud(cloud_0);
    icp_point.setInputTarget(align_cloud_h);
    icp_point.align (*align_cloud_i);

    //Visualize the point clouds using PCLVisualizer
    visualization::PCLVisualizer visualizer("Cloud Visualizer");
    visualizer.addPointCloud(align_cloud_a, "Align Cloud A");
    visualizer.addPointCloud(align_cloud_b, "Align Cloud B");
    visualizer.addPointCloud(align_cloud_c, "Align Cloud C");
    visualizer.addPointCloud(align_cloud_d, "Align Cloud D");
    visualizer.addPointCloud(align_cloud_e, "Align Cloud E");
    visualizer.addPointCloud(align_cloud_f, "Align Cloud F");
    visualizer.addPointCloud(align_cloud_g, "Align Cloud G");
    visualizer.addPointCloud(align_cloud_h, "Align Cloud H");
    visualizer.addPointCloud(align_cloud_i, "Align Cloud I");
    visualizer.addPointCloud(cloud_9, "Target Cloud");
    visualizer.spin();
    while (!visualizer.wasStopped()){}
    return 0;

    
}
