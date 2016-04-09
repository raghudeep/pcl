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
#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/spin_image.h>
//#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/rift.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/shot_omp.h>

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/features/features.hpp>

using namespace pcl::console;

#define NORM_EST_RAD 0.03 // 3 // Use all neighbors in a sphere of radius 3cm
#define FPFH_RAD_SEARCH 0.2
#define NUM_THREADS 64

struct Normal2PointXYZ
{
  pcl::PointXYZ operator()(const pcl::Normal& n) const
  {
    pcl::PointXYZ xyz;
    xyz.x = n.normal[0];
    xyz.y = n.normal[1];
    xyz.z = n.normal[2];
    return xyz;
  }
};

int
main (int, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  TicToc tt;
  tt.tic ();
  print_highlight ("Reading "); print_value ("%s \n", filename.c_str ());
  if (pcl::io::loadPCDFile <pcl::PointXYZRGB> (filename.c_str (), *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->points.size()); print_info (" points]\n");

  // Estimating the normals
  tt.tic ();
  print_highlight ("Computing normals using OpenMP for "); print_value ("%s \n", filename.c_str ());
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  normal_estimation.setSearchMethod (kdtree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_n (new pcl::PointCloud< pcl::Normal>);
  normal_estimation.setRadiusSearch (NORM_EST_RAD);
  normal_estimation.setNumberOfThreads(NUM_THREADS);
  normal_estimation.compute (*cloud_n);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud_n->points.size()); print_info (" points]\n");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *cloud2);
  std::vector<pcl::PointXYZ> normals_for_gpu(cloud_n->points.size());
  std::transform(cloud_n->points.begin(), cloud_n->points.end(), normals_for_gpu.begin(), Normal2PointXYZ()); 

  //Estimating spin on gpu
  tt.tic ();
  print_highlight ("Computing spin using GPU for "); print_value ("%s \n", filename.c_str ());
  pcl::gpu::SpinImageEstimation::PointCloud cloud_d(cloud_n->points.size());
  cloud_d.upload(cloud2->points);
  pcl::gpu::SpinImageEstimation::Normals normals_d;
  normals_d.upload(normals_for_gpu);
  pcl::gpu::SpinImageEstimation spin_image_descriptor(8, 0.5, 16);
  spin_image_descriptor.setInputCloud (cloud_d);
  spin_image_descriptor.setInputNormals (normals_d);
  spin_image_descriptor.setInputWithNormals (cloud_d, normals_d);
  spin_image_descriptor.setRadiusSearch (0.2, cloud_n->points.size());
  pcl::gpu::DeviceArray2D<pcl::Histogram<153> > spin_images_d;
  pcl::gpu::DeviceArray<unsigned char> mask_d;
  spin_image_descriptor.compute(spin_images_d, mask_d);
  std::vector<pcl::Histogram<153> > spin_images; int c;
  std::vector<unsigned char> mask;
  spin_images_d.download(spin_images, c);
  mask_d.download(mask);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", spin_images.size()); print_info (" points]\n");

  //std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;

  //pcl::PointCloud<>::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);

/*  // Use the same KdTree from the normal estimation
  spin_image_descriptor.setSearchMethod (kdtree);

  // Actually compute the spin images
  spin_image_descriptor.compute (*spin_images);

  // Display and retrieve the spin image descriptor vector for the first point.
  pcl::Histogram<153> first_descriptor = spin_images->points[0];
  std::cout << first_descriptor << std::endl;*/

  // Estimating Fast Point Feature Histogram
  /*tt.tic ();
  print_highlight ("Computing FPFH using OpenMP for "); print_value ("%s \n", filename.c_str ());
  pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
  fpfh_estimation.setInputCloud (cloud);
  fpfh_estimation.setInputNormals (cloud_n);
  fpfh_estimation.setSearchMethod (kdtree);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  fpfh_estimation.setRadiusSearch (FPFH_RAD_SEARCH);
  fpfh_estimation.setNumberOfThreads(NUM_THREADS);
  fpfh_estimation.compute (*pfh_features);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", pfh_features->points.size()); print_info (" points]\n");*/

  // Estimating SHOT
  /*tt.tic ();
  print_highlight ("Computing SHOT using OpenMP for "); print_value ("%s \n", filename.c_str ());
  pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot_estimation;
  shot_estimation.setRadiusSearch (FPFH_RAD_SEARCH);
  shot_estimation.setNumberOfThreads(NUM_THREADS);
  shot_estimation.setInputCloud (cloud);
  shot_estimation.setInputNormals (cloud_n);
  //shot_estimation.setSearchMethod (kdtree);
  pcl::PointCloud<pcl::SHOT352>::Ptr shot_features (new pcl::PointCloud<pcl::SHOT352>);
  //descr_est.setSearchSurface (modelPtr);
  shot_estimation.compute (*shot_features);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", shot_features->points.size()); print_info (" points]\n");*/

  // Estimating SHOT
  /*tt.tic ();
  print_highlight ("Computing SHOT using OpenMP for "); print_value ("%s \n", filename.c_str ());
  pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shotcolor_estimation;
  shotcolor_estimation.setRadiusSearch (FPFH_RAD_SEARCH);
  shotcolor_estimation.setNumberOfThreads(NUM_THREADS);
  shotcolor_estimation.setInputCloud (cloud);
  shotcolor_estimation.setInputNormals (cloud_n);
  pcl::PointCloud<pcl::SHOT1344>::Ptr shotcolor_features (new pcl::PointCloud<pcl::SHOT1344>);
  shotcolor_estimation.compute (*shotcolor_features);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", shotcolor_features->points.size()); print_info (" points]\n");*/

  // Estimating the normals GPU
  /*tt.tic ();
  print_highlight ("Computing normals using GPU for "); print_value ("%s \n", filename.c_str ());
  pcl::gpu::Octree::PointCloud cloud_device;
  cloud_device.upload(cloud->points);  
  pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
  octree_device->setCloud(cloud_device);
  octree_device->build();

  pcl::gpu::Feature::PointCloud cloud_d(cloud->width * cloud->height);
  cloud_d.upload(cloud->points);
  pcl::gpu::NormalEstimation ne_d;
  ne_d.setInputCloud(cloud_d);
  ne_d.setViewPoint(0, 0, 0);
  ne_d.setRadiusSearch(NORM_EST_RAD, 4);
  pcl::gpu::Feature::Normals normals_d(cloud->width * cloud->height);

  ne_d.compute(normals_d);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->points.size()); print_info (" points]\n");*/


  //pcl::gpu::NormalEstimation normal_estimation;
  /*normal_estimation.setInputCloud (cloud);
  normal_estimation.setRadiusSearch (NORM_EST_RAD);*/




  // Estimate the Intensity Gradient
/*  tt.tic ();
  print_highlight ("Computing normals for "); print_value ("%s \n", filename.c_str ());
  pcl::PointCloud<pcl::IntensityGradient>::Ptr cloud_ig (new pcl::PointCloud<pcl::IntensityGradient>);
  pcl::IntensityGradientEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::IntensityGradient> gradient_est;
  gradient_est.setInputCloud(cloud);
  gradient_est.setInputNormals(cloud_n);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treept2 (new pcl::search::KdTree<pcl::PointXYZRGB> (false));
  gradient_est.setSearchMethod(treept2);
  gradient_est.setRadiusSearch(0.25);
  gradient_est.compute(*cloud_ig);
  std::cout<<" Intesity Gradient estimated";
  std::cout<<" with size "<< cloud_ig->points.size() <<std::endl;*/

/*
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

  /*// Setup spin image computation
  pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor(8, 0.5, 16);
  spin_image_descriptor.setInputCloud (cloud);
  spin_image_descriptor.setInputNormals (normals);

  // Use the same KdTree from the normal estimation
  spin_image_descriptor.setSearchMethod (kdtree);
  pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
  spin_image_descriptor.setRadiusSearch (0.4); // default 0.2

  // Actually compute the spin images
  spin_image_descriptor.compute (*spin_images);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->points.size()); print_info (" points]\n");
  std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;*/

/*  pcl::FPFHSignature33 descriptor = pfh_features->points[0];
  std::cout << descriptor << std::endl;
  std::cout << pfh_features->points.size() << std::endl;

  std::stringstream ss;
  ss << filename.substr(0,filename.length()-4) << "_geo.txt";
  std::ofstream f(ss.str().c_str());

  tt.tic ();
  print_highlight ("Saving "); print_value ("%s ", ss.str().c_str ());
  for (int i=0; i < cloud->points.size(); i++) {
    f << cloud_n->points[i].normal_x << " " << cloud_n->points[i].normal_y << " " << cloud_n->points[i].normal_z << " ";
    pcl::FPFHSignature33 desc = pfh_features->points[i];
    for (int j=0; j < pfh_features->points.size(); j++)
      f << desc.histogram[j] << " ";
    / *pcl::Histogram<153> desc = spin_images->points[i];
    for (int j=0; j < spin_images->points.size(); j++)
      f << desc.histogram[j] << " ";* /
    f << std::endl;
  }
  f.close();
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->points.size()); print_info (" points]\n");*/
  return 0;
}
