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

#include <iostream>
#include <towr_plus/terrain/examples/height_map_examples.h>

namespace towr_plus {

FlatGround::FlatGround(double height) { height_ = height; }

double Block::GetHeight(double x, double y) const {
  double h = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <= block_start + eps_)
    h = slope_ * (x - block_start);

  if (block_start + eps_ <= x && x <= block_end - eps_)
    h = height_;

  if (block_end - eps_ <= x && x <= block_end)
    h = -slope_ * (x - (block_end - eps_)) + height_;

  return h;
}

double Block::GetHeightDerivWrtX(double x, double y) const {
  double dhdx = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <= block_start + eps_)
    dhdx = slope_;

  if (block_end - eps_ <= x && x <= block_end)
    dhdx = -slope_;

  return dhdx;
}

// STAIRS
double Stairs::GetHeight(double x, double y) const {
  double h = 0.0;

  if (x >= first_step_start_) {
    h = height_first_step;
  }

  if (x >= first_step_start_ + first_step_width_) {
    h = height_second_step;
  }

  if (x >= first_step_start_ + first_step_width_ + width_top) {
    h = 0.0;
  }

  return h;
}

// GAP
double Gap::GetHeight(double x, double y) const {
  double h = 0.0;

  // modelled as parabola
  if (gap_start_ <= x && x <= gap_end_x)
    h = a * x * x + b * x + c;

  return h;
}

double Gap::GetHeightDerivWrtX(double x, double y) const {
  double dhdx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dhdx = 2 * a * x + b;

  return dhdx;
}

double Gap::GetHeightDerivWrtXX(double x, double y) const {
  double dzdxx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dzdxx = 2 * a;

  return dzdxx;
}

// SLOPE
double Slope::GetHeight(double x, double y) const {
  double z = 0.0;
  if (x >= slope_start_)
    z = slope_ * (x - slope_start_);

  // going back down
  if (x >= x_down_start_) {
    z = height_center - slope_ * (x - x_down_start_);
  }

  // back on flat ground
  if (x >= x_flat_start_)
    z = 0.0;

  return z;
}

double Slope::GetHeightDerivWrtX(double x, double y) const {
  double dzdx = 0.0;
  if (x >= slope_start_)
    dzdx = slope_;

  if (x >= x_down_start_)
    dzdx = -slope_;

  if (x >= x_flat_start_)
    dzdx = 0.0;

  return dzdx;
}

// Chimney
double Chimney::GetHeight(double x, double y) const {
  double z = 0.0;

  if (x_start_ <= x && x <= x_end_)
    z = slope_ * (y - y_start_);

  return z;
}

double Chimney::GetHeightDerivWrtY(double x, double y) const {
  double dzdy = 0.0;

  if (x_start_ <= x && x <= x_end_)
    dzdy = slope_;

  return dzdy;
}

// Chimney LR
double ChimneyLR::GetHeight(double x, double y) const {
  double z = 0.0;

  if (x_start_ <= x && x <= x_end1_)
    z = slope_ * (y - y_start_);

  if (x_end1_ <= x && x <= x_end2_)
    z = -slope_ * (y + y_start_);

  return z;
}

double ChimneyLR::GetHeightDerivWrtY(double x, double y) const {
  double dzdy = 0.0;

  if (x_start_ <= x && x <= x_end1_)
    dzdy = slope_;

  if (x_end1_ <= x && x <= x_end2_)
    dzdy = -slope_;

  return dzdy;
}

///////////////////// Class Methods /////////////////////

double Travis::GetHeight(double x, double y) const
{
  // Converting Meter Space coordinates to Index Space
  double height = 0;
  double x_index = x/resolution;
  double y_index = y/resolution;

  // Obtain the Whole part and Decimal part of Index Space Coordinates
  double x_whole = std::floor(x_index);
  double x_fractional = x_index - x_whole;

  double y_whole = std::floor(y_index);
  double y_fractional = y_index - y_whole;

  // Handling out of bounds requests
  if(x_whole > x_length){x_whole = x_length-2; x_fractional = 1;}
  if(y_whole > y_length){y_whole = y_length-2; y_fractional = 1;}
  if(x_whole < 0){x_whole = 0; x_fractional = 0;}
  if(y_whole < 0){y_whole = 0; y_fractional = 0;}

  // Index Space Coordinates of surrounding points (P1)
  int p1_x = x_whole;
  int p1_y = y_whole;
  int p1_z = p1_y*x_length + p1_x;

  // Index Space Coordinates of surrounding points (P2)
  int p2_x = x_whole + 1;
  int p2_y = y_whole;
  int p2_z = p2_y*x_length + p2_x;

  // Index Space Coordinates of surrounding points (P3)
  int p3_x = x_whole;
  int p3_y = y_whole + 1;
  int p3_z = p3_y*x_length + p3_x;

  // Index Space Coordinates of surrounding points (P4)
  int p4_x = x_whole + 1;
  int p4_y = y_whole + 1;
  int p4_z = p4_y*x_length + p4_x;

  // Respective Heights (Z value in meters) for surrounding points
  double p1_height = zpos[p1_z];
  double p2_height = zpos[p2_z];
  double p3_height = zpos[p3_z];
  double p4_height = zpos[p4_z];

  // Height of desired point along the Y-Direction
  double delta_h_bottom = p2_height - p1_height;
  double delta_z_bottom = delta_h_bottom*x_fractional;
  double h_bottom = p1_height + delta_z_bottom;

  //Top Section
  double delta_h_top = p4_height - p3_height;
  double delta_z_top = delta_h_top*x_fractional;
  double h_top = p3_height + delta_z_top;

  //Desired Height Section
  double delta_h = h_top - h_bottom;
  double delta_z = delta_h*y_fractional;
  height = h_bottom + delta_z;

  ///// Uncomment the following block of code for debugging purposes /////

  // if(height > 0.1){
  //   std::cout << "\nDesired X and Y Coordinates: " << x << ", " << y << std::endl;
  //   std::cout << "Height: " << height << std::endl;
  // }
  // std::cout << "X Index: " << x_index << std::endl;
  // std::cout << "X Whole: " << x_whole << std::endl;
  // std::cout << "X Fraction: " << x_fractional << std::endl;
  // std::cout << "Y Index: " << y_index << std::endl;
  // std::cout << "Y Whole: " << y_whole << std::endl;
  // std::cout << "Y Fraction: " << y_fractional << std::endl;
  // std::cout << "\n" << std::endl;

  // std::cout << "1st Point Height: " << p1_height << std::endl;
  // std::cout << "2nd Point Height: " << p2_height << std::endl;
  // std::cout << "3rd Point Height: " << p3_height << std::endl;
  // std::cout << "4th Pount Height: " << p4_height << std::endl;
  // std::cout << "\n" << std::endl;

  // std::cout << "delta_h_bottom: " << delta_h_bottom << std::endl;
  // std::cout << "delta_z_bottom: " << delta_z_bottom << std::endl;
  // std::cout << "h1_bottom: " << h_bottom << std::endl;
  // std::cout << "\n" << std::endl;

  // std::cout << "delta_h_top: " << delta_h_top << std::endl;
  // std::cout << "delta_z_top: " << delta_z_top << std::endl;
  // std::cout << "h2_top: " << h_top << std::endl;
  // std::cout << "\n" << std::endl;
  
  // std::cout << "delta_h_des: " << delta_h << std::endl;
  // std::cout << "delta_z_des: " << delta_z << std::endl;
  // std::cout << "height: " << height << std::endl;
  // std::cout << "\n" << std::endl;
  return height;
}

double Travis::GetHeightDerivWrtY(double x, double y) const
{
  // Converting Meter Space coordinates to Index Space
  double dh_dy = 0;

  // Heights to Average (Z value in meters) for surrounding points
  double p1_height = GetHeight(x, y - resolution);
  double p2_height = GetHeight(x, y + resolution);

  //Derivative of Height Wrt to Y
  dh_dy =  (p2_height- p1_height) / (2*resolution); 

  return dh_dy;
}

double Travis::GetHeightDerivWrtX(double x, double y) const
{
  // Converting Meter Space coordinates to Index Space
  double dh_dx = 0;

  // Heights to Average (Z value in meters) for surrounding points
  double p1_height = GetHeight(x - resolution, y);
  double p2_height = GetHeight(x + resolution, y);

  //Derivative of Height Wrt to X
  dh_dx =  (p2_height- p1_height) / (2*resolution); // if h2_top > h1_bottom, then Positive slope. Else, Negative Slope

  return dh_dx;
}

// double Travis::GetHeightDerivWrtY(double x, double y) const
// {
//   // Handling out of bounds requests
//   if(x > xmax){x = xmax;}
//   if(y > ymax){return 0.0;}
//   if(x < xmin){x = xmin;}
//   if(y < ymin){return 0.0;}

//   // Converting Meter Space coordinates to Index Space
//   double dh_dy = 0;
//   double x_index = x/resolution;
//   double y_index = y/resolution;

//   // Obtain the Whole part and Decimal part of Index Space Coordinates
//   double x_whole = std::floor(x_index);
//   double x_fractional = x_index - x_whole;

//   double y_whole = std::floor(y_index);
//   double y_fractional = y_index - y_whole;

//   // x_fractional = std::modf(x_index, &x_whole);
//   // y_fractional = std::modf(y_index, &y_whole);

//   // Index Space Coordinates of surrounding points (P1)
//   int p1_x = x_whole;
//   int p1_y = y_whole;
//   int p1_z = p1_x*y_length + p1_y;

//   // Index Space Coordinates of surrounding points (P2)
//   int p2_x = x_whole + 1;
//   int p2_y = y_whole;
//   int p2_z = p2_x*y_length + p2_y;

//   // Index Space Coordinates of surrounding points (P3)
//   int p3_x = x_whole;
//   int p3_y = y_whole + 1;
//   int p3_z = p3_x*y_length + p3_y;

//   // Index Space Coordinates of surrounding points (P4)
//   int p4_x = x_whole + 1;
//   int p4_y = y_whole + 1;
//   int p4_z = p4_x*y_length + p4_y;

//   // Respective Heights (Z value in meters) for surrounding points
//   double p1_height = zpos[p1_z];
//   double p2_height = zpos[p2_z];
//   double p3_height = zpos[p3_z];
//   double p4_height = zpos[p4_z];


//   // Height of desired point along the Y-Direction

//   //Bottom Section
//   // double delta_h_bottom = fabs(p1_height - p2_height);
//   // double delta_x_bottom = x_fractional*resolution;
//   // double delta_z_bottom = delta_h_bottom*delta_x_bottom;
//   // double h1_bottom;
//   // if (p1_height > p2_height)
//   // {
//   //   h1_bottom = p1_height - delta_z_bottom;
//   // }
//   // else
//   // {
//   //   h1_bottom = p2_height - delta_z_bottom;
//   // }
//   double delta_h_bottom = p2_height - p1_height;
//   double delta_z_bottom = delta_h_bottom*x_fractional;
//   double h_bottom = p1_height + delta_z_bottom;

//   //Top Section
//   // double delta_h_top = fabs(p3_height - p4_height);
//   // double delta_x_top = x_fractional*resolution;
//   // double delta_z_top = delta_h_bottom*delta_x_bottom;
//   // double h2_top;
//   // if (p3_height > p4_height)
//   // {
//   //   h2_top = p3_height - delta_z_top;
//   // }
//   // else
//   // {
//   //   h2_top = p4_height - delta_z_top;
//   // }
//   double delta_h_top = p4_height - p3_height;
//   double delta_z_top = delta_h_top*x_fractional;
//   double h_top = p3_height + delta_z_top;

//   //Derivative of Height Wrt to Y
//   dh_dy =  (h_top - h_bottom) / resolution; // if h2_top > h1_bottom, then Positive slope. Else, Negative Slope

//   std::cout << "\nDesired X and Y Coordinates: " << x << ", " << y << std::endl;
//   std::cout << "Slope wrt Y: " << dh_dy << std::endl;
//   // std::cout << "1st Point Height: " << p1_height << std::endl;
//   // std::cout << "2nd Point Height: " << p2_height << std::endl;
//   // std::cout << "3rd Point Height: " << p3_height << std::endl;
//   // std::cout << "4th Pount Height: " << p4_height << std::endl;
//   // std::cout << "\n" << std::endl;

//   return dh_dy;
// }

// double Travis::GetHeightDerivWrtX(double x, double y) const
// {
//   // Handling out of bounds requests
//   if(x > xmax){return 0.0;}
//   if(y > ymax){y = ymax;}
//   if(x < xmin){return 0.0;}
//   if(y < ymin){y = ymin;}

//   // Converting Meter Space coordinates to Index Space
//   double dh_dx = 0;
//   double x_index = x/resolution;
//   double y_index = y/resolution;

//   // Obtain the Whole part and Decimal part of Index Space Coordinates
//   double x_whole = std::floor(x_index);
//   double x_fractional = x_index - x_whole;

//   double y_whole = std::floor(y_index);
//   double y_fractional = y_index - y_whole;

//   // x_fractional = std::modf(x_index, &x_whole);
//   // y_fractional = std::modf(y_index, &y_whole);

//   // Index Space Coordinates of surrounding points (P1)
//   int p1_x = x_whole;
//   int p1_y = y_whole;
//   int p1_z = p1_x*y_length + p1_y;

//   // Index Space Coordinates of surrounding points (P2)
//   int p2_x = x_whole + 1;
//   int p2_y = y_whole;
//   int p2_z = p2_x*y_length + p2_y;

//   // Index Space Coordinates of surrounding points (P3)
//   int p3_x = x_whole;
//   int p3_y = y_whole + 1;
//   int p3_z = p3_x*y_length + p3_y;

//   // Index Space Coordinates of surrounding points (P4)
//   int p4_x = x_whole + 1;
//   int p4_y = y_whole + 1;
//   int p4_z = p4_x*y_length + p4_y;

//   // Respective Heights (Z value in meters) for surrounding points
//   double p1_height = zpos[p1_z];
//   double p2_height = zpos[p2_z];
//   double p3_height = zpos[p3_z];
//   double p4_height = zpos[p4_z];

//   // Height of desired point along the Y-Direction

//   //Bottom Section
//   // double delta_h_left = fabs(p1_height - p3_height);
//   // double delta_y_left = y_fractional*resolution;
//   // double delta_z_left = delta_h_left*delta_y_left;
//   // double h1_left;
//   // if (p1_height > p3_height)
//   // {
//   //   h1_left = p1_height - delta_z_left;
//   // }
//   // else
//   // {
//   //   h1_left = p3_height - delta_z_left;
//   // }
//   double delta_h_left = p3_height - p1_height;
//   double delta_z_left = delta_h_left*y_fractional;
//   double h_left = p1_height + delta_z_left;

//   //Top Section
//   // double delta_h_right = fabs(p2_height - p4_height);
//   // double delta_y_right = y_fractional*resolution;
//   // double delta_z_right = delta_h_right*delta_y_right;
//   // double h2_right;
//   // if (p2_height > p4_height)
//   // {
//   //   h2_right = p2_height - delta_z_right;
//   // }
//   // else
//   // {
//   //   h2_right = p4_height - delta_z_right;
//   // }
//   double delta_h_right = p4_height - p2_height;
//   double delta_z_right = delta_h_right*y_fractional;
//   double h_right = p2_height + delta_z_right;

//   //Derivative of Height Wrt to Y
//   dh_dx =  (h_right - h_left) / resolution; // if h2_top > h1_bottom, then Positive slope. Else, Negative Slope

//   // std::cout << "\nDesired X and Y Coordinates: " << x << ", " << y << std::endl;
//   // std::cout << "Slope wrt X: " << dh_dx << std::endl;

//   // std::cout << "1st Point Height: " << p1_height << std::endl;
//   // std::cout << "2nd Point Height: " << p2_height << std::endl;
//   // std::cout << "3rd Point Height: " << p3_height << std::endl;
//   // std::cout << "4th Pount Height: " << p4_height << std::endl;
//   // std::cout << "\n" << std::endl;

//   return dh_dx;
// }

/////////////////////////////////////////////////////////


} /* namespace towr_plus */
