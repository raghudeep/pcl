/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <sstream>

#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.txt output.pcd xyzirgb\n", argv[0]);
}

bool
loadCloudXYZ (const string &filename, PointCloud<PointXYZ> &cloud)
{
  ifstream fs;
  fs.open (filename.c_str (), ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno)); 
    fs.close ();
    return (false);
  }
  
  string line;
  vector<string> st;

  while (!fs.eof ())
  {
    getline (fs, line);
    // Ignore empty lines
    if (line == "")
      continue;

    // Tokenize the line
    boost::trim (line);
    boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

    if (st.size () != 7)
      continue;

    cloud.push_back (PointXYZ (float (atof (st[0].c_str ())), float (atof (st[1].c_str ())), float (atof (st[2].c_str ()))));
  }
  fs.close ();

  cloud.width = uint32_t (cloud.size ()); cloud.height = 1; cloud.is_dense = true;
  return (true);
}

bool
loadCloudXYZI (const string &filename, PointCloud<PointXYZI> &cloud)
{
  ifstream fs;
  fs.open (filename.c_str (), ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno)); 
    fs.close ();
    return (false);
  }
  
  string line;
  vector<string> st;

  while (!fs.eof ())
  {
    getline (fs, line);
    // Ignore empty lines
    if (line == "")
      continue;

    // Tokenize the line
    boost::trim (line);
    boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

    if (st.size () != 7)
      continue;

    PointXYZI xyzi_point;
    xyzi_point.x = float (atof (st[0].c_str ()));
    xyzi_point.y = float (atof (st[1].c_str ()));
    xyzi_point.z = float (atof (st[2].c_str ()));
    xyzi_point.intensity = float (atof (st[3].c_str ()));

    cloud.push_back ( xyzi_point );
  }
  fs.close ();

  cloud.width = uint32_t (cloud.size ()); cloud.height = 1; cloud.is_dense = true;
  return (true);
}

bool
loadCloudXYZRGB (const string &filename, PointCloud<PointXYZRGB> &cloud)
{
  ifstream fs;
  fs.open (filename.c_str (), ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno)); 
    fs.close ();
    return (false);
  }
  string line;
  vector<string> st;

  while (!fs.eof ())
  {
    getline (fs, line);
    // Ignore empty lines
    if (line == "")
      continue;

    // Tokenize the line
    boost::trim (line);
    boost::split (st, line, boost::is_any_of ("\t \r "), boost::token_compress_on);

    if (st.size() != 7)
      continue;

    float x = atof(st[0].c_str());
    float y = atof(st[1].c_str());
    float z = atof(st[2].c_str());

    float r = float(atof(st[4].c_str()));
    float g = float(atof(st[5].c_str()));
    float b = float(atof(st[6].c_str()));

    /*PointXYZRGB xyzrgb_point; // = PointXYZRGB(uint8_t(r), g, b);
    xyzrgb_point.x = x; 
    xyzrgb_point.y = y;
    xyzrgb_point.z = z;
    xyzrgb_point.rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);*/

    PointXYZRGB xyzrgb_point = PointXYZRGB(uint8_t(r), uint8_t(g), uint8_t(b));
    xyzrgb_point.x = x; 
    xyzrgb_point.y = y;
    xyzrgb_point.z = z;
    cloud.push_back ( xyzrgb_point );
  }
  fs.close ();

  cloud.width = uint32_t (cloud.size ()); cloud.height = 1; cloud.is_dense = true;
  return (true);
}

bool
loadCloudXYZRGBA (const string &filename, PointCloud<PointXYZRGBA> &cloud)
{
  ifstream fs;
  fs.open (filename.c_str (), ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno)); 
    fs.close ();
    return (false);
  }
  string line;
  vector<string> st;

  while (!fs.eof ())
  {
    getline (fs, line);
    // Ignore empty lines
    if (line == "")
      continue;

    // Tokenize the line
    boost::trim (line);
    boost::split (st, line, boost::is_any_of ("\t \r "), boost::token_compress_on);

    if (st.size() != 7)
      continue;

    float x = atof(st[0].c_str());
    float y = atof(st[1].c_str());
    float z = atof(st[2].c_str());

    float r = float(atof(st[4].c_str()));
    float g = float(atof(st[5].c_str()));
    float b = float(atof(st[6].c_str()));

    PointXYZRGBA xyzrgba_point; // = PointXYZRGB(uint8_t(r), g, b);
    xyzrgba_point.x = x; 
    xyzrgba_point.y = y;
    xyzrgba_point.z = z;
    xyzrgba_point.rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    /*PointXYZRGBA xyzrgba_point = PointXYZRGBA(uint8_t(r), uint8_t(g), uint8_t(b));
    xyzrgb_point.x = x; 
    xyzrgb_point.y = y;
    xyzrgb_point.z = z;*/

    cloud.push_back ( xyzrgba_point );
  }
  fs.close ();

  cloud.width = uint32_t (cloud.size ()); cloud.height = 1; cloud.is_dense = true;
  return (true);
}


/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 4)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .ply files
  vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  vector<int> txt_file_indices = parse_file_extension_argument (argc, argv, ".txt");
  if (pcd_file_indices.size () != 1 || txt_file_indices.size () != 1)
  {
    print_error ("Need one input XYZ file and one output PCD file.\n");
    return (-1);
  }

  std::cout << "Converting " << argv[txt_file_indices[0]] << "\n";
  TicToc tt; tt.tic();
  // Load the first file
  if (strcmp(argv[3], "xyz") == 0) {
    PointCloud<PointXYZ> cloud;
    if (!loadCloudXYZ (argv[txt_file_indices[0]], cloud)) 
      return (-1);
    //savePCDFileASCII (argv[pcd_file_indices[0]], cloud);
    PCDWriter w;
    w.writeBinaryCompressed (argv[pcd_file_indices[0]], cloud);
  } else if (strcmp(argv[3], "xyzi") == 0) {
    PointCloud<PointXYZI> cloud;
    if (!loadCloudXYZI (argv[txt_file_indices[0]], cloud)) 
      return (-1);
    //savePCDFileASCII (argv[pcd_file_indices[0]], cloud);
    PCDWriter w;
    w.writeBinaryCompressed (argv[pcd_file_indices[0]], cloud);
  } else if (strcmp(argv[3], "xyzrgb") == 0) {
    PointCloud<PointXYZRGB> cloud;
    if (!loadCloudXYZRGB (argv[txt_file_indices[0]], cloud)) 
      return (-1);
    //savePCDFileASCII (argv[pcd_file_indices[0]], cloud);
    PCDWriter w;
    w.writeBinaryCompressed (argv[pcd_file_indices[0]], cloud);
  } else if (strcmp(argv[3], "xyzrgba") == 0) {
    std::cout << "check code for xyzrgba\n";
    return (-1);
    PointCloud<PointXYZRGBA> cloud;
    if (!loadCloudXYZRGBA (argv[txt_file_indices[0]], cloud)) 
      return (-1);
    //savePCDFileASCII (argv[pcd_file_indices[0]], cloud);
    PCDWriter w;
    w.writeBinaryCompressed (argv[pcd_file_indices[0]], cloud);
  }
  std::cout << "Converted in " << tt.toc() << " ms.\n";

  return 0;
}

