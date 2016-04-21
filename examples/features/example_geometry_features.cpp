/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */



#include <iostream>
#include <vector>
#include <fstream>

#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

#define NORM_EST_RAD 0.03 
#define FPFH_RAD_SEARCH 0.2
#define SHOT_RAD_SEARCH 0.2
#define NUM_THREADS 64

int
main (int, char**)
{
  std::string filename = "/home/raghudeep/varcity3dchallenge/data/ruemonge428/pcl.ply";
  std::string train_filename = "/home/raghudeep/varcity3dchallenge/data/ruemonge428/pcl_gt_train.ply";
  std::string test_filename = "/home/raghudeep/varcity3dchallenge/data/ruemonge428/pcl_gt_test.ply";

  TicToc tt;

  pcl::PLYReader reader;
  pcl::PCLPointCloud2 blob;
  tt.tic ();
  print_highlight ("Reading point clouds ... "); print_value ("%s\n", filename.c_str ());
  if (reader.read (filename, blob) < 0) return (false);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2 (blob, *cloud_xyzrgb);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normal (new pcl::PointCloud<pcl::Normal>);
  pcl::fromPCLPointCloud2 (blob, *cloud_normal);
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (*cloud_xyzrgb).c_str ());
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (*cloud_normal).c_str ());

  print_highlight ("Reading point clouds ... "); print_value ("%s\n", train_filename.c_str ());
  if (reader.read (train_filename, blob) < 0) return (false);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_train (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2 (blob, *cloud_train);
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList ( *cloud_train ).c_str ());

  print_highlight ("Reading point clouds ... "); print_value ("%s\n", test_filename.c_str ());
  if (reader.read (test_filename, blob) < 0) return (false);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_test (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2 (blob, *cloud_test);
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (*cloud_test).c_str ());
  print_info ("done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud_xyzrgb->width * cloud_xyzrgb->height); print_info (" points]\n");

  pcl::search::KdTree<PointXYZRGB>::Ptr kdtree;
  kdtree.reset (new search::KdTree<PointXYZRGB> (false));
  pcl::search::KdTree<PointXYZ>::Ptr kdtree2;
  kdtree2.reset (new search::KdTree<PointXYZ> (false));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);

  // indices of training and testing points.
  pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
  for (int i=0; i < cloud_xyzrgb->points.size(); i++) 
    if ( !(cloud_train->points[i].r==0 && cloud_train->points[i].g==0 && cloud_train->points[i].b==0) ||
         !(cloud_test->points[i].r==0  && cloud_test->points[i].g==0  && cloud_test->points[i].b==0) ) 
      indices->indices.push_back(i);
  print_info("Number of key points : %d\n",indices->indices.size());

  // Estimating the normals
  /*tt.tic ();
  print_highlight ("Computing normals for "); print_value ("%s ", filename.c_str ());
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud (cloud_xyz);
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation.setSearchMethod (kdtree2);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_n (new pcl::PointCloud< pcl::Normal>);
  normal_estimation.setRadiusSearch (NORM_EST_RAD);
  normal_estimation.setNumberOfThreads(NUM_THREADS);
  //normal_estimation.compute (*cloud_n);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud_n->points.size()); print_info (" points]\n");*/

  // Estimating Fast Point Feature Histogram
  tt.tic ();
  print_highlight ("Computing FPFH using OpenMP for "); print_value ("%s \n", filename.c_str ());
  pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
  fpfh_estimation.setInputCloud (cloud_xyzrgb);
  fpfh_estimation.setInputNormals (cloud_normal);
  fpfh_estimation.setSearchMethod (kdtree);
  fpfh_estimation.setNumberOfThreads(NUM_THREADS);
  fpfh_estimation.setIndices(indices);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  fpfh_estimation.setRadiusSearch (0.3);
  fpfh_estimation.compute (*pfh_features);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", pfh_features->points.size()); print_info (" points]\n");

  tt.tic ();
  print_highlight ("Computing SHOTColor using OpenMP for "); print_value ("%s \n", filename.c_str ());
  pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shotcolor_estimation;
  shotcolor_estimation.setRadiusSearch (0.3);
  shotcolor_estimation.setNumberOfThreads(NUM_THREADS);
  shotcolor_estimation.setInputCloud (cloud_xyzrgb);
  shotcolor_estimation.setInputNormals (cloud_normal);
  shotcolor_estimation.setIndices(indices);
  pcl::PointCloud<pcl::SHOT1344>::Ptr shotcolor_features (new pcl::PointCloud<pcl::SHOT1344>);
  shotcolor_estimation.compute (*shotcolor_features);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", shotcolor_features->points.size()); print_info (" points]\n");

  tt.tic ();
  print_highlight ("Computing SpinImages for "); print_value ("%s \n", filename.c_str ());
  pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor(8, 0.0, 0);
  spin_image_descriptor.setInputCloud (cloud_xyz);
  spin_image_descriptor.setInputNormals (cloud_normal);
  spin_image_descriptor.setSearchMethod (kdtree2);
  pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
  spin_image_descriptor.setRadiusSearch (0.3); // default 0.2
  spin_image_descriptor.setIndices(indices);
  spin_image_descriptor.compute (*spin_images);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", spin_images->points.size()); print_info (" points]\n");

  std::ofstream test_("test.txt");
  std::ofstream train_("train.txt");

  tt.tic ();
  print_highlight ("Saving ");
  for (int i=0; i < indices->indices.size(); i++) {
    int I = indices->indices[i];
    if ( !(cloud_train->points[I].r==0 && cloud_train->points[I].g==0 && cloud_train->points[I].b==0) ) {
      for (int j=0; j < 33; j++)
        train_ << pfh_features->points[i].histogram[j] << " ";
      for (int j=0; j < 153; j++)
        train_ << spin_images->points[i].histogram[j] << " ";
      if (shotcolor_features->points[i].descriptorSize() == 1344) {
        for (int j=0; j < 1344; j++)
          train_ << shotcolor_features->points[i].descriptor[j] << " ";
      } else {
        for (int j=0; j < 1344; j++)
          train_ << " 0 ";
      }
      train_ << "\n";
    }
    else if ( !(cloud_test->points[I].r==0  && cloud_test->points[I].g==0  && cloud_test->points[I].b==0) ) {
      for (int j=0; j < 33; j++)
        test_ << pfh_features->points[i].histogram[j] << " ";
      for (int j=0; j < 153; j++)
        test_ << spin_images->points[i].histogram[j] << " ";
      if (shotcolor_features->points[i].descriptorSize() == 1344) {
        for (int j=0; j < 1344; j++)
          test_ << shotcolor_features->points[i].descriptor[j] << " ";
      } else {
        for (int j=0; j < 1344; j++)
          test_ << " 0 ";
      }
      test_ << "\n";
    } 
  }
  train_.close(); test_.close();
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud_xyzrgb->points.size()); print_info (" points]\n");


  /*// Estimate the Intensity Gradient
  tt.tic ();
  print_highlight ("Computing normals for "); print_value ("%s ", filename.c_str ());
  pcl::PointCloud<pcl::IntensityGradient>::Ptr cloud_ig (new pcl::PointCloud<pcl::IntensityGradient>);
  pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> gradient_est;
  gradient_est.setInputCloud(cloud);
  gradient_est.setInputNormals(cloud_n);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr treept2 (new pcl::search::KdTree<pcl::PointXYZI> (false));
  gradient_est.setSearchMethod(treept2);
  gradient_est.setRadiusSearch(0.25);
  gradient_est.compute(*cloud_ig);
  std::cout<<" Intesity Gradient estimated";
  std::cout<<" with size "<< cloud_ig->points.size() <<std::endl;


  // Estimate the RIFT feature
  pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, pcl::Histogram<32> > rift_est;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr treept3 (new pcl::search::KdTree<pcl::PointXYZI> (false));
  rift_est.setSearchMethod(treept3);
  rift_est.setRadiusSearch(10.0);
  rift_est.setNrDistanceBins (4);
  rift_est.setNrGradientBins (8);
  rift_est.setInputCloud(cloud);
  rift_est.setInputGradient(cloud_ig);
  pcl::PointCloud<pcl::Histogram<32> > rift_output;
  rift_est.compute(rift_output);

  std::cout<<" RIFT feature estimated";
  std::cout<<" with size "<<rift_output.points.size()<<std::endl;
  
  // Display and retrieve the rift descriptor vector for the first point
  pcl::Histogram<32> first_descriptor = rift_output.points[0];
  std::cout << first_descriptor << std::endl;*/

  return 0;
}
