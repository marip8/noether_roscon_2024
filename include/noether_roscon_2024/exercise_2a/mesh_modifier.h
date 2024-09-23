#pragma once

#include <noether_tpp/core/mesh_modifier.h>

namespace noether
{
/**
 * @brief Mesh modifier that segments the input mesh into cylinders aligned with the z-axis of the mesh
 */
class CylinderSegmentationMeshModifier : public MeshModifier
{
public:
  /**
   * @brief Constructor with all parameters needed to segment cylinders from an input mesh
   */
  CylinderSegmentationMeshModifier(float min_radius,
                                   float max_radius,
                                   float distance_threshold,
                                   float axis_threshold = 10.0 * M_PI / 180.0,
                                   float normal_distance_weight = 0.1,
                                   unsigned min_vertices = 1,
                                   int max_cylinders = -1,
                                   unsigned max_iterations = 100);

  /**
   * @brief Overrides the virtual modification function to segment cylinder shapes out of the input mesh
   */
  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  /** @brief Minimum required cylinder radius (m) */
  float min_radius_;
  /** @brief Maximum required cylinder radius (m) */
  float max_radius_;
  /** @brief Maximum distance (m) a point can be from a model of a cylinder to be considered an inlier */
  float distance_threshold_;
  /** @brief Maximum angle (radians) by which the axis of a detected cylinder can differ from the axis of the defined model cylinder */
  float axis_threshold_;
  /** @brief Distance weighting amount given to normals (vs point positions) when comparing to the cylinder model (value from [0, 1]) */
  float normal_distance_weight_;
  /** @brief Minimum number of vertices that a cluster (identfied as a cylinder) must have */
  unsigned min_vertices_;
  /** @brief Maximum number of cylinders to detect */
  int max_cylinders_;
  /** @brief Maximum number of RANSAC iterations to perform */
  unsigned max_iterations_;
};

} // namespace noether
