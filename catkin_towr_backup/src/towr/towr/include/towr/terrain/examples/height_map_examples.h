#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_

#include <towr/terrain/height_map.h>

namespace towr {

/**
 * @addtogroup Terrains
 * @{
 */

/**
 * @brief Sample terrain of even height.
 */


class FlatGround : public HeightMap {
public:
  FlatGround(double desired_height = 0.0) : height_(desired_height) {}   // 원하는 높이를 초기화합니다. 필요에 따라 이 값을 변경할 수 있습니다.
  double GetHeight(double x, double y) const override {
    return height_;
  }
  // GetDerivativeOfHeight 메서드는 이전과 동일하게 유지합니다.

  double GetHeightDerivWrtX(double x, double y) const override {
    return 0.0;  // 평평한 지형의 x 방향 도함수는 0
  }

  double GetHeightDerivWrtY(double x, double y) const override {
    return 0.0;  // 평평한 지형의 y 방향 도함수는 0
  }

private:
  double height_;  // 원하는 높이를 저장하는 멤버 변수입니다.
};


class SlopeWithRollPitch : public HeightMap {
public:
  SlopeWithRollPitch(double height, double roll, double pitch);

  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;
  double GetHeightDerivWrtY(double x, double y) const override;

private:
  double height_;
  double roll_;  // roll angle in radians
  double pitch_; // pitch angle in radians
  FlatGround base_ground_; // The base ground for reference
};




class SlopeWithImpulse : public SlopeWithRollPitch {
public:
  SlopeWithImpulse(double height, double roll, double pitch)
    : SlopeWithRollPitch(height, roll, pitch) {}

  double GetHeight(double x, double y) const override {
    // 기본적인 경사면 높이를 가져옴
    double h = SlopeWithRollPitch::GetHeight(x, y);

    // 박스형 장애물의 위치와 크기를 정의
    double box_start_x = 1.0; // 예를 들어, x=2에서 시작
    double box_end_x = 1.3;   // 예를 들어, x=4에서 끝
    double box_height = 0.2;

    // x 좌표가 박스 장애물 범위 내에 있는지 확인
    if (x >= box_start_x && x <= box_end_x) {
      h += box_height;
    }

    return h;
  }
};



class SlopeWithStep: public SlopeWithRollPitch {
public:
  SlopeWithStep(double height, double roll, double pitch)
    : SlopeWithRollPitch(height, roll, pitch) {}

  double GetHeight(double x, double y) const override {
    // 기본적인 경사면 높이를 가져옴
    double h = SlopeWithRollPitch::GetHeight(x, y);

    // 박스형 장애물의 위치와 크기를 정의
    double box_start_x = 1.0; // 예를 들어, x=2에서 시작
    double box_end_x = 3.5;   // 예를 들어, x=4에서 끝
    double box_height = 0.3;

    // x 좌표가 박스 장애물 범위 내에 있는지 확인
    // if (x >= box_start_x && x <= box_end_x) {
    //   h += box_height;
    // }

    if (x >= box_start_x) {
      h += box_height;
    }


    return h;
  }
};


class SlopeWithSmallPit : public SlopeWithRollPitch {
public:
  SlopeWithSmallPit(double height, double roll, double pitch) : SlopeWithRollPitch(height, roll, pitch) {}

  double GetHeight(double x, double y) const override {
    // 기본적인 경사면 높이를 가져옴
    double h = SlopeWithRollPitch::GetHeight(x, y);

    // 박스형 장애물의 위치와 크기를 정의
    double box_start_x = 1.0; // 예를 들어, x=2에서 시작
    double box_end_x = 1.15;   // 예를 들어, x=4에서 끝
    double box_height = -0.8;

    // x 좌표가 박스 장애물 범위 내에 있는지 확인
    if (x >= box_start_x && x <= box_end_x) {
      h += box_height;
    }
   
    return h;
  }   
};



class SlopeWithLargePit : public SlopeWithRollPitch {
public:
  SlopeWithLargePit(double height, double roll, double pitch) : SlopeWithRollPitch(height, roll, pitch) {}

  double GetHeight(double x, double y) const override {
    // 기본적인 경사면 높이를 가져옴
    double h = SlopeWithRollPitch::GetHeight(x, y);

    // 박스형 장애물의 위치와 크기를 정의
    double box_start_x = 1.0; // 예를 들어, x=2에서 시작
    double box_end_x = 2.0;   // 예를 들어, x=4에서 끝
    double box_height = -0.3;

    // x 좌표가 박스 장애물 범위 내에 있는지 확인
    if (x >= box_start_x && x <= box_end_x) {
      h += box_height;
    }

    return h;
  }  
};


/**
 * @brief Sample terrain with a step in height in x-direction.
 */
// class Block : public HeightMap {
// public:
//   double GetHeight(double x, double y)  const override;
//   double GetHeightDerivWrtX(double x, double y) const override;

// private:
//   double block_start = 0.7;
//   double length_     = 3.5;
//   double height_     = 0.5; // [m]

//   double eps_ = 0.03; // approximate as slope
//   const double slope_ = height_/eps_;
// };

/**
 * @brief Sample terrain with a two-steps in height in x-direction.
 */
class Stairs : public HeightMap {
public:
  double GetHeight(double x, double y) const override;

private:
  double first_step_start_  = 1.0;
  double first_step_width_  = 1.0;
  double height_first_step  = 0.2;
  double height_second_step = 0.4;
  double width_top = 1.0;
};

// Slope terrain __TEST
class Slope : public HeightMap {
public:
  double GetHeight(double x, double y) const override;
  double GetHeightDerivWrtX(double x, double y) const override;

private:
  double x_start_ = 1.0;  // x 방향으로 경사가 시작되는 지점
  double x_end_ = 4.0;    // x 방향으로 경사가 끝나는 지점
  double x_obstacle_start = 1.5;  // 장애물 시작 지점
  double x_obstacle_end = 2.5;    // 장애물 끝 지점

  double slope_ = 0.5;    // 경사의 정도 (높이 변화량 / x 변화량)
};

// /**
//  * @brief Sample terrain with parabola-modeled gap in x-direction.
//  */
// class Gap : public HeightMap {
// public:
//   double GetHeight(double x, double y) const override;
//   double GetHeightDerivWrtX(double x, double y) const override;
//   double GetHeightDerivWrtXX(double x, double y) const override;

// private:
//   const double gap_start_ = 1.0;
//   const double w = 0.5;
//   const double h = 1.5;

//   const double slope_ = h/w;
//   const double dx = w/2.0;
//   const double xc = gap_start_ + dx; // gap center
//   const double gap_end_x = gap_start_ + w;

//   // generated with matlab
//   // see matlab/gap_height_map.m
//   // coefficients of 2nd order polynomial
//   // h = a*x^2 + b*x + c
//   const double a = (4*h)/(w*w);
//   const double b = -(8*h*xc)/(w*w);
//   const double c = -(h*(w - 2*xc)*(w + 2*xc))/(w*w);
// };

// /**
//  * @brief Sample terrain with an increasing and then decreasing slope in x-direction.
//  */
// class Slope : public HeightMap {
// public:
//   double GetHeight(double x, double y) const override;
//   double GetHeightDerivWrtX(double x, double y) const override;

// private:
//   const double slope_start_ = 1.0;
//   const double up_length_   = 1.0;
//   const double down_length_ = 1.0;
//   const double height_center = 0.7;

//   const double x_down_start_ = slope_start_+up_length_;
//   const double x_flat_start_ = x_down_start_ + down_length_;
//   const double slope_ = height_center/up_length_;
// };

// /**
//  * @brief Sample terrain with a tilted vertical wall to cross a gap.
//  */
// class Chimney : public HeightMap {
// public:
//   double GetHeight(double x, double y) const override;
//   double GetHeightDerivWrtY(double x, double y) const override;

// private:
//   const double x_start_ = 1.0;
//   const double length_  = 1.5;
//   const double y_start_ = 0.5; // distance to start of slope from center at z=0
//   const double slope_   = 3.0;

//   const double x_end_ = x_start_+length_;
// };

// /**
//  * @brief Sample terrain with two tilted vertical walls to cross a gap.
//  */
// class ChimneyLR : public HeightMap {
// public:
//   double GetHeight(double x, double y) const override;
//   double GetHeightDerivWrtY(double x, double y) const override;

// private:
//   const double x_start_ = 0.5;
//   const double length_  = 1.0;
//   const double y_start_ = 0.5; // distance to start of slope from center at z=0
//   const double slope_   = 2;

//   const double x_end1_ = x_start_+length_;
//   const double x_end2_ = x_start_+2*length_;
// };

/** @}*/

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEIGHT_MAP_EXAMPLES_H_ */