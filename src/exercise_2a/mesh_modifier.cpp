#include <noether_roscon_2024/exercise_2a/mesh_modifier.h>

// Include SAC segmentation from PCL
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/conversions.h>

// Include function from `noether` for extracting a sub-mesh from vertex inliers
#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>

// Include for std::iota
#include <numeric>

namespace noether
{
CylinderSegmentationMeshModifier::CylinderSegmentationMeshModifier(float min_radius,
                                                                   float max_radius,
                                                                   float distance_threshold,
                                                                   float axis_threshold,
                                                                   float normal_distance_weight,
                                                                   unsigned min_vertices,
                                                                   int max_cylinders,
                                                                   unsigned max_iterations)
  : MeshModifier()
  , min_radius_(min_radius)
  , max_radius_(max_radius)
  , distance_threshold_(distance_threshold)
  , axis_threshold_(axis_threshold)
  , normal_distance_weight_(normal_distance_weight)
  , min_vertices_(min_vertices)
  , max_cylinders_(max_cylinders)
  , max_iterations_(max_iterations)
{
}

std::vector<pcl::PolygonMesh> CylinderSegmentationMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Convert the mesh vertices to a point cloud
  auto cloud_points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud_points);

  // Convert the mesh vertex normals to a point cloud of normals
  auto cloud_normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud_normals);

  // Set up the output data structure
  std::vector<pcl::PolygonMesh> output;

  // Set up the RANSAC cylinder fit model
  auto model = pcl::make_shared<pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>>(cloud_points);
  model->setInputNormals(cloud_normals);
  model->setNormalDistanceWeight(normal_distance_weight_);
  model->setRadiusLimits(min_radius_, max_radius_);
  model->setAxis(Eigen::Vector3f::UnitZ());
  model->setEpsAngle(axis_threshold_);

  auto ransac = pcl::make_shared<pcl::RandomSampleConsensus<pcl::PointXYZ>>(model);
  ransac->setDistanceThreshold(distance_threshold_);
  ransac->setMaxIterations(max_iterations_);

  // Create a vector of indices for the remaining indices to which a cylinder model can be fit
  // To start, all indices are remaining (i.e., [0, 1, 2, ..., cloud->size() - 1])
  std::vector<int> remaining_indices(cloud_points->size());
  std::iota(remaining_indices.begin(), remaining_indices.end(), 0);

  /* Detect as many cylinders as possible while:
   *   - there are at least enough vertices left to form a new model cluster, and
   *   - we haven't detected more than the maximum number of cylinders
   */
  while (remaining_indices.size() >= min_vertices_ && output.size() < max_cylinders_)
  {
    // Fit a cylinder model to the vertices using RANSAC
    model->setIndices(remaining_indices);
    ransac->computeModel();

    // Extract the inliers and ensure there are enough to form a valid model cluster
    std::vector<int> inliers;
    ransac->getInliers(inliers);
    if (inliers.size() < min_vertices_)
      break;

    // Extract the inlier submesh
    pcl::PolygonMesh output_mesh = extractSubMeshFromInlierVertices(mesh, inliers);
    if (!output_mesh.polygons.empty())
    {
      // TODO: Project the inlier vertices onto the cylinder

      // Append the extracted and projected mesh to the vector of output meshes
      output.push_back(output_mesh);
    }

    // Remove the inlier indices from the list of remaining indices
    std::size_t num_outliers = remaining_indices.size() - inliers.size();
    if (num_outliers < min_vertices_)
      break;

    std::vector<int> outliers;
    outliers.reserve(num_outliers);
    std::set_difference(remaining_indices.begin(),
                        remaining_indices.end(),
                        inliers.begin(),
                        inliers.end(),
                        std::back_inserter(outliers));
    remaining_indices = outliers;
  }

  return output;
}

} // namespace noether
