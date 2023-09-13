/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/******************************************************************************
Modified by Junhyeok Ahn (junhyeokahn91@gmail.com) for towr+
******************************************************************************/

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_

#include <towr_plus/terrain/height_map.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <vector>
#include <iterator>
#include <cstdlib>


namespace towr_plus {

/**
 * @addtogroup Terrains
 * @{
 */

/**
 * @brief Sample terrain of even height.
 */
class FlatGround : public HeightMap {
public:
  FlatGround(double height = 0.0);
  double GetHeight(double x, double y) const override { return height_; };

private:
  double height_; // [m]
};

/**
 * @brief Sample terrain with a step in height in x-direction.
 */
class Block : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;

private:
  double block_start = -0.15;
  double length_ = 0.3;
  double height_ = 0.3; // [m]
  double block_end = block_start + length_;

  double eps_ = 0.03; // approximate as slope
  const double slope_ = height_ / eps_;
};

/**
 * @brief Sample terrain with a two-steps in height in x-direction.
 */
class Stairs : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

private:
  double first_step_start_ = 0.2;
  double first_step_width_ = 0.35;
  double height_first_step = 0.1;
  double height_second_step = 0.2;
  double width_top = 0.35;
};

/**
 * @brief Sample terrain with parabola-modeled gap in x-direction.
 */
class Gap : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;
  double GetHeightDerivWrtXX(double x, double y) const override;

private:
  const double gap_start_ = 0.4;
  const double w = 0.1;
  const double h = 1.5;

  const double slope_ = h / w;
  const double dx = w / 2.0;
  const double xc = gap_start_ + dx; // gap center
  const double gap_end_x = gap_start_ + w;

  // generated with matlab
  // see matlab/gap_height_map.m
  // coefficients of 2nd order polynomial
  // h = a*x^2 + b*x + c
  const double a = (4 * h) / (w * w);
  const double b = -(8 * h * xc) / (w * w);
  const double c = -(h * (w - 2 * xc) * (w + 2 * xc)) / (w * w);
};

/**
 * @brief Sample terrain with an increasing and then decreasing slope in
 * x-direction.
 */
class Slope : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;

private:
  const double slope_start_ = 0.15;
  const double up_length_ = 0.3;
  const double down_length_ = 0.3;
  const double height_center = 0.05;

  const double x_down_start_ = slope_start_ + up_length_;
  const double x_flat_start_ = x_down_start_ + down_length_;
  const double slope_ = height_center / up_length_;
};

/**
 * @brief Sample terrain with a tilted vertical wall to cross a gap.
 */
class Chimney : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtY(double x, double y) const override;

private:
  const double x_start_ = 0.2;
  const double length_ = 0.3;
  const double y_start_ = -0.2; // distance to start of slope from center at z=0
  const double slope_ = 0.4;

  const double x_end_ = x_start_ + length_;
};

/**
 * @brief Sample terrain with two tilted vertical walls to cross a gap.
 */
class ChimneyLR : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtY(double x, double y) const override;

private:
  // const double x_start_ = 0.2;
  const double x_start_ = 0.3;
  const double length_ = 0.3;
  const double y_start_ = -0.2; // distance to start of slope from center at z=0
  const double slope_ = 0.4;

  const double x_end1_ = x_start_ + length_;
  const double x_end2_ = x_start_ + 2 * length_;
};

/**
 * @brief Terrain from height map file.
 */
class Travis : public HeightMap 
{
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtY(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;

  //////// CONSTRUCTOR //////////////////////////////////////////

  // Takes in a string file path to the .txt file containing height data of a height map
  Travis(std::string data_filename) 
  {
    std::fstream mapfile;
    mapfile.open(data_filename, std::ios::in);

    std::string line;
    std::string number_as_string;

    xmax, ymax, xmin, ymin, zmax, zmin = 0;

    while (std::getline(mapfile, line, '\n'))
    {
        int i = 0;

        float x,y,z;

        for (std::istringstream ss(line); std::getline(ss, number_as_string, ',');)
        {
          // std::cout << line << '\n';
          if (i == 0)
          {
            x = std::stof(number_as_string);
            xpos.push_back(x);
            if (x > xmax){xmax = x;}
            if (x < xmin){xmin = x;}
          }
          else if (i == 1)
          {
            y = std::stof(number_as_string);
            ypos.push_back(y);
            if (y > ymax){ymax = y;}
            if (y < ymin){ymin = y;}
          }
          else if (i == 2)
          {
            z = std::stof(number_as_string);
            zpos.push_back(z);
            if (z > zmax){zmax = z;}
            if (z < zmin){zmin = z;}
          }
                
          i++;
        }
              
    }
    // Uncomment the following block of code for debugging purposes /////

    // std::cout << "X Position: " ;
    // for (int i = 0; i < xpos.size(); i++ )
    // {
    //   std::cout << xpos[i] << ", ";
    // }

    // std::cout << "\nY position: ";

    // for (int i = 0; i < ypos.size(); i++ )
    // {
    //   std::cout << ypos[i] << ", ";
    // }

    // std::cout << "\nZ position: ";
    // for (int i = 0; i < zpos.size(); i++ )
    // {
    //   std::cout << zpos[i] << ", ";
    // }
    // std::cout << "\n";

    mapfile.close();

    data_vector_size = xpos.size();
    resolution = xpos[1] - xpos[0];

    x_length = std::round((xmax-xmin)/resolution) + 1;
    y_length = std::round((ymax-ymin)/resolution) + 1;

    // std::cout << "\nzmax: " << zmax << "\nzmin: " << zmin << std::endl;
    // std::cout << "\nxmin: " << xmin << "\nymin: "<< ymin << "\nx length: " << x_length << "\ny length: " << y_length << std::endl;
  }
  //////////////////////////////////////////////////////////

private:
  ////////// Class Attributes ///////////////////////////////////

  // Vectors containing X and Y positions along with height Data (Z)
  std::vector<float> xpos;
  std::vector<float> ypos;
  std::vector<float> zpos;

  float xmax, ymax, xmin, ymin, zmax, zmin;

  // Variable that outputs size of vectors containing x,y, and z.
  int data_vector_size;
  int x_length;
  int y_length;

  // Variable that outputs point resolution
  double resolution; 

  //////////////////////////////////////////////////////////////
};


/** @}*/

} /* namespace towr_plus */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_ */
