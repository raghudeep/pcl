#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int
main (int argc, char** argv)
{
  std::string filename;
  bool input_file_specified = pcl::console::find_switch (argc, argv, "-i");
  if (input_file_specified)
    pcl::console::parse (argc, argv, "-i", filename);
  else
    return -1;

  float leafsize = 0.1f;
  bool leafsize_specified = pcl::console::find_switch (argc, argv, "-r");
  if (leafsize_specified)
    pcl::console::parse (argc, argv, "-r", leafsize);

  std::string out_path = filename.substr(0,filename.find_last_of('.'))+".index";
  bool output_file_specified = pcl::console::find_switch (argc, argv, "-o");
  if (output_file_specified)
    pcl::console::parse (argc, argv, "-o", out_path);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
 
  TicToc tt; tt.tic ();
  print_highlight ("Reading "); print_value ("%s \n", filename.c_str ());
  if (pcl::io::loadPCDFile <pcl::PointXYZRGB> (filename.c_str (), *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->points.size()); print_info (" points]\n");

  // Create the filtering object
  print_highlight ("Voxelizing... "); print_value ("%s \n", filename.c_str ());
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (leafsize, leafsize, leafsize);
  sor.setSaveLeafLayout(true);
  sor.filter (*cloud_filtered);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud_filtered->points.size()); print_info (" points]\n");

  print_highlight ("Initializing labels... "); print_value ("%s \n", filename.c_str ()); tt.tic();
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZL>);
  pcl::copyPointCloud(*cloud, *cloud2);
  print_info("done,  "); print_value("%d", cloud2->points.size()); print_info(", in "); print_value("%g ms\n", tt.toc());

  print_highlight ("Unique labels... "); print_value ("%s \n", filename.c_str ()); tt.tic();
  std::set<int> unique_labels;
  //#pragma omp parallel for
  for (int i=0; i<cloud->width * cloud->height; i++) {
    //std::cout << sor.getCentroidIndexAt(sor.getGridCoordinates(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z)) << "\n";
    cloud2->points[i].label = sor.getCentroidIndex(cloud->points[i]);
    unique_labels.insert(cloud2->points[i].label);
  }
  print_info("done,  "); print_value("%d", unique_labels.size()); print_info(", in "); print_value("%g ms\n", tt.toc());

  print_highlight ("Saving label cloud for... "); print_value ("%s \n", filename.c_str ()); tt.tic();
  PCDWriter w;
  w.writeBinaryCompressed ("index.pcd", *cloud2);
  print_info("done,  "); print_value("%d", cloud2->points.size()); print_info(", in "); print_value("%g ms\n", tt.toc());

  print_highlight ("Creating label map... "); print_value ("%s \n", filename.c_str ());
  tt.tic(); std::map<int, int> label_map; int count = 0;
  for (std::set<int>::iterator it=unique_labels.begin(); it!=unique_labels.end(); ++it)
  {
     label_map[*it] = count;
     count++;
  }
  print_info("done,  "); print_value("%d", count); print_info(", in "); print_value("%g ms\n", tt.toc());

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
      int indx = cloud2->points[i].label;
      if (indx!=0) frequency[label_map[indx]][gt_label]++;
    }
    in_.open (gt_path.c_str());
    for (int i=0; i<cloud->width * cloud->height; i++)
    { 
      int indx = label_map[cloud2->points[i].label];
      //int gt_label; in_ >> gt_label;
      //if (indx==0) out_ << gt_label << "\n";
      //else out_ << std::distance(frequency[indx], std::max_element(frequency[indx], frequency[indx] + 9)) << "\n";
      out_ << std::distance(frequency[indx], std::max_element(frequency[indx], frequency[indx] + num_classes)) << "\n";
    }
    std::cout << "Writing sp gt file : " << out_path << " in " << tt.toc() << " ms.\n";
  }
  else
  {
    for (int i=0; i<cloud->width * cloud->height; i++)
      out_ << cloud2->points[i].label << "\n";
    std::cout << "Writing index file : " << out_path << " in " << tt.toc() << " ms.\n";
  }
  out_.close();

  return (0);
}
