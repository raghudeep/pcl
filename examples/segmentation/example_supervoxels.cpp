#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

// Types
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    pcl::console::print_info ("Syntax is: %s {-p <pcd-file> OR -r <rgb-file> -d <depth-file>} \n --NT  (disables use of single camera transform) \n -o <output-file> \n -O <refined-output-file> \n-l <output-label-file> \n -L <refined-output-label-file> \n-v <voxel resolution> \n-s <seed resolution> \n-c <color weight> \n-z <spatial weight> \n-n <normal_weight>] \n", argv[0]);
    return (1);
  }
  
  ///////////////////////////////  //////////////////////////////
  //////////////////////////////  //////////////////////////////
  ////// THIS IS ALL JUST INPUT HANDLING - Scroll down until 
  ////// pcl::SupervoxelClustering<pcl::PointXYZRGB> super
  //////////////////////////////  //////////////////////////////
  PointCloudT::Ptr cloud = boost::make_shared < PointCloudT >();
  bool pcd_file_specified = pcl::console::find_switch (argc, argv, "-p");
  std::string pcd_path;
    if (!pcd_file_specified)
    {
      std::cout << "No cloud specified!";
      return (1);
    }else
    {
      pcl::console::parse (argc,argv,"-p",pcd_path);
    }
  
  //bool disable_transform = pcl::console::find_switch (argc, argv, "--NT");
  
  std::string out_path;
  bool output_file_specified = pcl::console::find_switch (argc, argv, "-o");
  if (output_file_specified)
    pcl::console::parse (argc, argv, "-o", out_path);
  else
    out_path = "test_output.png";
  
  std::string out_label_path;
  bool output_label_file_specified = pcl::console::find_switch (argc, argv, "-l");
  if (output_label_file_specified)
    pcl::console::parse (argc, argv, "-l", out_label_path);
  else
    out_label_path = "test_output_labels.png";
  
  float voxel_resolution = 0.008f;
  bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (voxel_res_specified)
    pcl::console::parse (argc, argv, "-v", voxel_resolution);
    
  float seed_resolution = 0.08f;
  bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (seed_res_specified)
    pcl::console::parse (argc, argv, "-s", seed_resolution);
  
  float color_importance = 0.2f;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", color_importance);
  
  float spatial_importance = 0.4f;
  if (pcl::console::find_switch (argc, argv, "-z"))
    pcl::console::parse (argc, argv, "-z", spatial_importance);
  
  float normal_importance = 1.0f;
  if (pcl::console::find_switch (argc, argv, "-n"))
    pcl::console::parse (argc, argv, "-n", normal_importance);
  
  std::cout << "Loading pointcloud... " << pcd_path << "\n";
  std::cout << "v : " << voxel_resolution << ", s :" << seed_resolution << "\n";
  pcl::io::loadPCDFile (pcd_path, *cloud);
  for (PointCloudT::iterator cloud_itr = cloud->begin (); cloud_itr != cloud->end (); ++cloud_itr)
    if (cloud_itr->z < 0)
      cloud_itr->z = std::abs (cloud_itr->z);
  
  std::cout << "Done making cloud with " << cloud->width*cloud->height << " points!\n";

  ///////////////////////////////  //////////////////////////////
  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////
  //////////////////////////////  //////////////////////////////
  
  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  super.setUseSingleCameraTransform(false);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
 
  TicToc tt; tt.tic();
  std::cout << "Extracting supervoxels!\n";
  super.extract (supervoxel_clusters);
  std::cout << "Found " << supervoxel_clusters.size () << " Supervoxels\n";
  std::cout << "Supervoxels in " << tt.toc() << " ms.\n";

  /*tt.tic();
  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > refined_supervoxel_clusters;
  std::cout << "Refining supervoxels \n";
  super.refineSupervoxels (3, refined_supervoxel_clusters);
  std::cout << "Refined " << refined_supervoxel_clusters.size () << " Supervoxels!\n";
  std::cout << "Refined in " << tt.toc() << " ms.\n";*/

  PointLCloudT::Ptr full_labeled_cloud = super.getLabeledCloud ();
  std::cout << "Number of points in fully labelled supervoxel cloud " << full_labeled_cloud->width * full_labeled_cloud->height << "!\n";
  std::cout << "Max label : " << super.getMaxLabel() << "\n";

  tt.tic();
  int count = 0; std::set<int> unique_labels;
  //#pragma omp parallel for
  for (int i=0; i<full_labeled_cloud->width * full_labeled_cloud->height; i++)
    if (full_labeled_cloud->points[i].label==0) count++;
    else unique_labels.insert(full_labeled_cloud->points[i].label);
  std::cout << "Number of unlabelled points : " << count << "! and unique labels : " << unique_labels.size() << " in " << tt.toc() << " ms.\n";

  tt.tic();
  std::map<int, int> label_map; count = 0;
  for (std::set<int>::iterator it=unique_labels.begin(); it!=unique_labels.end(); ++it)
  {
     label_map[*it] = count;
     count++;
  }
  std::cout << "Creating label map = " << count << " in " << tt.toc() << " ms.\n";

  tt.tic();
  bool gt_file_specified = pcl::console::find_switch (argc, argv, "-g");
  std::ofstream out_(out_path.c_str());
  if (gt_file_specified) 
  {
    std::string gt_path;
    pcl::console::parse (argc, argv, "-g", gt_path);
    int num_classes = 9;
    int frequency[unique_labels.size()][num_classes];
    for (int i=0; i<unique_labels.size(); i++) for (int j=0; j<num_classes; j++) frequency[i][j] = 0;
    std::ifstream in_(gt_path.c_str());
    for (int i=0; i<cloud->width * cloud->height; i++)
    {
      int gt_label; in_ >> gt_label;
      int indx = full_labeled_cloud->points[i].label;
      if (indx!=0) frequency[label_map[indx]][gt_label]++;
    }
    in_.open (gt_path.c_str());
    for (int i=0; i<cloud->width * cloud->height; i++)
    { 
      int indx = label_map[full_labeled_cloud->points[i].label];
      //int gt_label; in_ >> gt_label;
      //if (indx==0) out_ << gt_label << "\n";
      //else out_ << std::distance(frequency[indx], std::max_element(frequency[indx], frequency[indx] + 9)) << "\n";
      out_ << std::distance(frequency[indx], std::max_element(frequency[indx], frequency[indx] + 9)) << "\n";
    }
    std::cout << "Writing sp gt file : " << out_path << " in " << tt.toc() << " ms.\n";
  }
  else
  {
    for (int i=0; i<full_labeled_cloud->width * full_labeled_cloud->height; i++)
      out_ << full_labeled_cloud->points[i].label << "\n";
    std::cout << "Writing index file : " << out_path << " in " << tt.toc() << " ms.\n";
  }
  out_.close();
  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr full_colored_cloud = super.getColoredCloud ();
  //PCDWriter w;
  //w.writeBinaryCompressed (out_path, *full_colored_cloud);
  //savePCDFileASCII(out_path, *full_labeled_cloud);
  /*PointCloudT::Ptr refined_colored_voxel_cloud = super.getColoredVoxelCloud ();
  PointNCloudT::Ptr refined_sv_normal_cloud = super.makeSupervoxelNormalCloud (refined_supervoxel_clusters);
  PointLCloudT::Ptr refined_full_labeled_cloud = super.getLabeledCloud ();
  PointCloudT::Ptr refined_full_colored_cloud = super.getColoredCloud ();*/

  return (0);
}
