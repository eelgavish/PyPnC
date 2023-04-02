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

  ////////// Class Attributes ///////////////////////////////////

  // Vectors containing X and Y positions along with height Data (Z)
  std::vector<float> xpos;
  std::vector<float> ypos;
  std::vector<float> zpos;

  // Variable that outputs size of vectors containing x,y, and z.
  int data_vector_size;

  // Variable that outputs point resolution
  double resolution; 

  //////////////////////////////////////////////////////////////




  //////// CONSTRUCTOR //////////////////////////////////////////

  // Takes in a string file path to the .txt file containing height data of a height map
  Travis(std::string data_filename) 
  {
    std::fstream mapfile;
    mapfile.open(data_filename, std::ios::in);


    std::string line;
    std::string number_as_string;

    while (std::getline(mapfile, line, '\n'))
    {
            
        int i = 0;

        for (std::istringstream ss(line); std::getline(ss, number_as_string, ',');)
        {
          if (i == 0)
          {
            xpos.push_back(std::stof(number_as_string));
          }
          else if (i == 1)
          {
            ypos.push_back(std::stof(number_as_string));
          }
          else if (i == 2)
          {
            zpos.push_back(std::stof(number_as_string));
          }
                
                
          i++;
        }

              
    }

    data_vector_size = xpos.size();
    resolution = ypos[1] - ypos[0];
    

    ///// Uncomment the following block of code for debugging purposes /////

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

    };
    //////////////////////////////////////////////////////////





    ///////////////////// Class Methods /////////////////////

   double GetHeight(double x, double y) const override 
   {
    // Converting Meter Space coordinates to Index Space
    double height;
    double x_index = x/resolution;
    double y_index = y/resolution;


    // Obtain the Whole part and Decimal part of Index Space Coordinates
    double x_whole;
    double x_fractional;

    double y_whole;
    double y_fractional;

    x_fractional = std::modf(x_index, &x_whole);
    y_fractional = std::modf(y_index, &y_whole);

    // Index Space Coordinates of surrounding points (P1)
    int p1_x = x_whole;
    int p1_y = y_whole;
    int p1_z = p1_x*100 + p1_y;

    // Index Space Coordinates of surrounding points (P2)
    int p2_x = x_whole + 1;
    int p2_y = y_whole;
    int p2_z = p2_x*100 + p2_y;

    // Index Space Coordinates of surrounding points (P3)
    int p3_x = x_whole;
    int p3_y = y_whole + 1;
    int p3_z = p3_x*100 + p3_y;

    // Index Space Coordinates of surrounding points (P4)
    int p4_x = x_whole + 1;
    int p4_y = y_whole + 1;
    int p4_z = p4_x*100 + p4_y;

    // Respective Heights (Z value in meters) for surrounding points
    double p1_height = zpos[p1_z];
    double p2_height = zpos[p2_z];
    double p3_height = zpos[p3_z];
    double p4_height = zpos[p4_z];


    // Height of desired point along the Y-Direction
    
    //Bottom Section
    double delta_h_bottom = fabs(p1_height - p2_height);
    double delta_x_bottom = x_fractional*resolution;
    double delta_z_bottom = delta_h_bottom*delta_x_bottom;
    double h1_bottom;
    if (p1_height > p2_height)
    {
      h1_bottom = p1_height - delta_z_bottom;
    }
    else
    {
      h1_bottom = p2_height - delta_z_bottom;
    }


    //Top Section
    double delta_h_top = fabs(p3_height - p4_height);
    double delta_x_top = x_fractional*resolution;
    double delta_z_top = delta_h_bottom*delta_x_bottom;
    double h2_top;
    if (p3_height > p4_height)
    {
      h2_top = p3_height - delta_z_top;
    }
    else
    {
      h2_top = p4_height - delta_z_top;
    }


    //Desired Height Section
    double delta_h_des = fabs(h1_bottom - h2_top);
    double delta_y_des = y_fractional*resolution;
    double delta_z_des = delta_h_des*delta_y_des;
    if (h1_bottom > h2_top)
    {
      height = h1_bottom - delta_z_des;
    }
    else
    {
      height = h2_top - delta_z_des;
    }



    ///// Uncomment the following block of code for debugging purposes /////

        // std::cout << "Desired X and Y Coordinates: " << x << ", " << y << std::endl;
        // std::cout << "\n" << std::endl;


        // std::cout << "X Whole: " << x_whole << std::endl;
        // std::cout << "X Fraction: " << x_fractional << std::endl;
        // std::cout << "Y Whole: " << y_whole << std::endl;
        // std::cout << "Y Fraction: " << y_fractional << std::endl;
        // std::cout << "\n" << std::endl;

        // std::cout << "1st Point Height: " << p1_height << std::endl;
        // std::cout << "2nd Point Height: " << p2_height << std::endl;
        // std::cout << "3rd Point Height: " << p3_height << std::endl;
        // std::cout << "4th Pount Height: " << p4_height << std::endl;
        // std::cout << "\n" << std::endl;

        // std::cout << "delta_h_bottom: " << delta_h_bottom << std::endl;
        // std::cout << "delta_x_bottom: " << delta_x_bottom << std::endl;
        // std::cout << "delta_z_bottom: " << delta_z_bottom << std::endl;
        // std::cout << "h1_bottom: " << h1_bottom << std::endl;
        // std::cout << "\n" << std::endl;

        // std::cout << "delta_h_top: " << delta_h_top << std::endl;
        // std::cout << "delta_x_top: " << delta_x_top << std::endl;
        // std::cout << "delta_z_top: " << delta_z_top << std::endl;
        // std::cout << "h2_top: " << h2_top << std::endl;
        // std::cout << "\n" << std::endl;
        
        // std::cout << "delta_h_des: " << delta_h_des << std::endl;
        // std::cout << "delta__y_des: " << delta_y_des << std::endl;
        // std::cout << "delta_z_des: " << delta_z_des << std::endl;
        // std::cout << "h3_des: " << h3_des << std::endl;
        // std::cout << "\n" << std::endl;
    return height;
   }


   double GetDerivWrtY(double x, double y) const override
   {
    // Converting Meter Space coordinates to Index Space
    double dh_dy;
    double x_index = x/resolution;
    double y_index = y/resolution;


    // Obtain the Whole part and Decimal part of Index Space Coordinates
    double x_whole;
    double x_fractional;

    double y_whole;
    double y_fractional;

    x_fractional = std::modf(x_index, &x_whole);
    y_fractional = std::modf(y_index, &y_whole);

    // Index Space Coordinates of surrounding points (P1)
    int p1_x = x_whole;
    int p1_y = y_whole;
    int p1_z = p1_x*100 + p1_y;

    // Index Space Coordinates of surrounding points (P2)
    int p2_x = x_whole + 1;
    int p2_y = y_whole;
    int p2_z = p2_x*100 + p2_y;

    // Index Space Coordinates of surrounding points (P3)
    int p3_x = x_whole;
    int p3_y = y_whole + 1;
    int p3_z = p3_x*100 + p3_y;

    // Index Space Coordinates of surrounding points (P4)
    int p4_x = x_whole + 1;
    int p4_y = y_whole + 1;
    int p4_z = p4_x*100 + p4_y;

    // Respective Heights (Z value in meters) for surrounding points
    double p1_height = zpos[p1_z];
    double p2_height = zpos[p2_z];
    double p3_height = zpos[p3_z];
    double p4_height = zpos[p4_z];


    // Height of desired point along the Y-Direction
    
    //Bottom Section
    double delta_h_bottom = fabs(p1_height - p2_height);
    double delta_x_bottom = x_fractional*resolution;
    double delta_z_bottom = delta_h_bottom*delta_x_bottom;
    double h1_bottom;
    if (p1_height > p2_height)
    {
      h1_bottom = p1_height - delta_z_bottom;
    }
    else
    {
      h1_bottom = p2_height - delta_z_bottom;
    }


    //Top Section
    double delta_h_top = fabs(p3_height - p4_height);
    double delta_x_top = x_fractional*resolution;
    double delta_z_top = delta_h_bottom*delta_x_bottom;
    double h2_top;
    if (p3_height > p4_height)
    {
      h2_top = p3_height - delta_z_top;
    }
    else
    {
      h2_top = p4_height - delta_z_top;
    }


    //Derivative of Height Wrt to Y
    dh_dy =  (h2_top - h1_bottom) / resolution; // if h2_top > h1_bottom, then Positive slope. Else, Negative Slope

    return dh_dy;
   }


   double GetDerivWrtX(double x, double y) const override
   {
    // Converting Meter Space coordinates to Index Space
    double dh_dx;
    double x_index = x/resolution;
    double y_index = y/resolution;


    // Obtain the Whole part and Decimal part of Index Space Coordinates
    double x_whole;
    double x_fractional;

    double y_whole;
    double y_fractional;

    x_fractional = std::modf(x_index, &x_whole);
    y_fractional = std::modf(y_index, &y_whole);

    // Index Space Coordinates of surrounding points (P1)
    int p1_x = x_whole;
    int p1_y = y_whole;
    int p1_z = p1_x*100 + p1_y;

    // Index Space Coordinates of surrounding points (P2)
    int p2_x = x_whole + 1;
    int p2_y = y_whole;
    int p2_z = p2_x*100 + p2_y;

    // Index Space Coordinates of surrounding points (P3)
    int p3_x = x_whole;
    int p3_y = y_whole + 1;
    int p3_z = p3_x*100 + p3_y;

    // Index Space Coordinates of surrounding points (P4)
    int p4_x = x_whole + 1;
    int p4_y = y_whole + 1;
    int p4_z = p4_x*100 + p4_y;

    // Respective Heights (Z value in meters) for surrounding points
    double p1_height = zpos[p1_z];
    double p2_height = zpos[p2_z];
    double p3_height = zpos[p3_z];
    double p4_height = zpos[p4_z];


    // Height of desired point along the Y-Direction
    
    //Bottom Section
    double delta_h_left = fabs(p1_height - p3_height);
    double delta_y_left = y_fractional*resolution;
    double delta_z_left = delta_h_left*delta_y_left;
    double h1_left;
    if (p1_height > p3_height)
    {
      h1_left = p1_height - delta_z_left;
    }
    else
    {
      h1_left = p3_height - delta_z_left;
    }


    //Top Section
    double delta_h_right = fabs(p2_height - p4_height);
    double delta_y_right = y_fractional*resolution;
    double delta_z_right = delta_h_right*delta_y_right;
    double h2_right;
    if (p2_height > p4_height)
    {
      h2_right = p2_height - delta_z_right;
    }
    else
    {
      h2_right = p4_height - delta_z_right;
    }


    //Derivative of Height Wrt to Y
    dh_dx =  (h2_right - h1_left) / resolution; // if h2_top > h1_bottom, then Positive slope. Else, Negative Slope

    return dh_dx;
   }


    /////////////////////////////////////////////////////////

private:
}    


/** @}*/

} /* namespace towr_plus */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_ */
