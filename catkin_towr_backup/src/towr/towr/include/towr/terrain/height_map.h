
#ifndef TOWR_HEIGHT_MAP_H_
#define TOWR_HEIGHT_MAP_H_

#include <memory>
#include <vector>
#include <map>
#include <string>

#include <Eigen/Dense>

#include <towr/variables/cartesian_dimensions.h>

namespace towr {

/**
 * @defgroup Terrains
 * @brief The heightmap and slope of various terrains.
 *
 * #### Further Reading
 *  * The base class HeightMap is what all terrain depend on.
 *
 * \ref include/towr/terrain
 */

/**
 * @brief Holds the height and slope information of the terrain.
 *
 * This class is responsible for providing the height values and slope at
 * each position (x,y). Examples of various height map examples can be found
 * in height_map_examples.h.
 *
 * If a height map of the terrain already exists, e.g. Octomap/Gridmap, then
 * a simple adapter for these can be written to comply to  this minimal
 * interface and to be used with %towr.
 *
 * The height map is used to formulate constraints such as
 * "foot must be touching terrain during stance phase".
 * @sa TerrainConstraint
 *
 * @ingroup Terrains
 */


class HeightMap {
public:
  using Ptr      = std::shared_ptr<HeightMap>;
  using Vector3d = Eigen::Vector3d;

  /**
   * @brief Terrains IDs corresponding for factory method.
   */
  enum TerrainID { FlatID,
                  // CombinedTerrainID,
                  SlopeWithRollPitchID,
                  SlopeWithImpulseID,
                  SlopeWithStepID,
                  SlopeWithSmallGapID,
                  SlopeWithLargeGapID,
                  StairsID,
                  SlopeID,
                  //  GapID,
                  //  SlopeID,
                  //  ChimneyID,
                  //  ChimneyLRID,
                   TERRAIN_COUNT };

  static HeightMap::Ptr MakeTerrain(TerrainID type);
  
  enum Direction { Normal, Tangent1, Tangent2 };

  HeightMap() = default;
  virtual ~HeightMap () = default;

  /**
   * @returns The height of the terrain [m] at a specific 2D position.
   * @param x The x position.
   * @param y The y position.
   */
  virtual double GetHeight(double x, double y) const = 0;

  /**
   * @brief How the height value changes at a 2D position in direction dim.
   * @param dim  The direction (x,y) w.r.t. which the height change is desired.
   * @param x  The x position on the terrain.
   * @param y  The y position on the terrain.
   * @return  The derivative of the height with respect to the dimension.
   */
  double GetDerivativeOfHeightWrt(Dim2D dim, double x, double y) const;

  /**
   * @brief Returns either the vector normal or tangent to the terrain patch.
   * @param direction  The terrain normal or tangent vectors.
   * @param x  The x position on the terrain.
   * @param y  The y position on the terrain.
   * @return The normalized 3D vector in the specified direction.
   */
  Vector3d GetNormalizedBasis(Direction direction, double x, double y) const;

  /**
   * @brief How the terrain normal/tangent vectors change when moving in x or y.
   * @param direction  The terrain normal or tangent vectors.
   * @param dim  The dimension w.r.t which the change is searched for.
   * @param x  The x position on the terrain.
   * @param y  The y position on the terrain.
   * @return The normalized 3D derivative w.r.t dimension dim.
   */
  Vector3d GetDerivativeOfNormalizedBasisWrt(Direction direction, Dim2D dim,
                                             double x, double y) const;
  /**
   * @returns The constant friction coefficient over the whole terrain.
   */
  double GetFrictionCoeff() const { return friction_coeff_; };

protected:
  double friction_coeff_ = 0.5;

private:
  using DimDerivs = std::vector<Dim2D>; ///< dimensional derivatives
  /**
   * @brief returns either the terrain normal/tangent or its derivative.
   * @param direction Terrain normal or tangent vector.
   * @param x The x position on the terrain.
   * @param y The y position on the terrain.
   * @param dim_deriv If empty, the vector is returned, if e.g. X_ is set, the
   *                  derivative of the vector w.r.t. x is returned.
   * @returns the 3D @b not-normalized vector.
   */
  Vector3d GetBasis(Direction direction, double x, double y,
                    const DimDerivs& dim_deriv= {}) const;

  Vector3d GetNormal(double x,   double y, const DimDerivs& = {}) const;
  Vector3d GetTangent1(double x, double y, const DimDerivs& = {}) const;
  Vector3d GetTangent2(double x, double y, const DimDerivs& = {}) const;

  double GetSecondDerivativeOfHeightWrt(Dim2D dim1, Dim2D dim2,
                                        double x, double y) const;

  Vector3d GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(
      const Vector3d& non_normalized, int index) const;

  // first derivatives that must be implemented by the user
  virtual double GetHeightDerivWrtX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtY(double x, double y) const { return 0.0; };

  // second derivatives with respect to first letter, then second
  virtual double GetHeightDerivWrtXX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtXY(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtYX(double x, double y) const { return 0.0; };
  virtual double GetHeightDerivWrtYY(double x, double y) const { return 0.0; };
};


const static std::map<HeightMap::TerrainID, std::string> terrain_names =
{
  {HeightMap::FlatID,                     "Flat"       },
  // {HeightMap::CombinedTerrainID,        "Combined"       },
  {HeightMap::SlopeWithRollPitchID,        "Slope_With_RollPitch"       },
  {HeightMap::SlopeWithImpulseID,       "Slope_With_RollPitch_Impulse"       },
  {HeightMap::SlopeWithStepID,          "Slope_With_RollPitch_Step"       },
  {HeightMap::SlopeWithSmallGapID,      "Slope_With_RollPitch_Small_Gap"       },
  {HeightMap::SlopeWithLargeGapID,      "Slope_With_RollPitch_Large_Gap"       },
  {HeightMap::StairsID,                   "Stairs"     },
  {HeightMap::SlopeID,                   "Slope"     },
  // {HeightMap::GapID,                      "Gap"        },
  // {HeightMap::SlopeID,                    "Slope"      },
  // {HeightMap::ChimneyID,                  "Chimney"    },
  // {HeightMap::ChimneyLRID,                "ChimenyLR"  }
};

} /* namespace towr */

#endif /* TOWR_HEIGHT_MAP_H_ */
