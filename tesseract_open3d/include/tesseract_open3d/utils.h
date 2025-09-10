#ifndef TESSERACT_OPEN3D_UTILS_H
#define TESSERACT_OPEN3D_UTILS_H

// Open3d
#include <open3d/t/geometry/PointCloud.h>

namespace tesseract_o3d
{
/**
 * @brief combinePointCloud
 * Appends `input` into `output` (tensor API).
 *  - Keeps everything on output’s device (moves `input` if needed).
 *  - Always updates positions.
 *  - Keeps colors/normals consistent (pads with zeros as needed).
 *  - Extend the pattern for custom attrs via SetPointAttr/GetPointAttr.
 * @param output The output point cloud to combine input into
 * @param input The input point cloud to fuse into the output point cloud
 * @param custom_attributes Custom attributes to include in the fuse
 */
void combinePointCloud(open3d::t::geometry::PointCloud& output,
                       const open3d::t::geometry::PointCloud& input,
                       const std::vector<std::string>& custom_attributes = {});
}  // namespace tesseract_o3d

#endif  // TESSERACT_OPEN3D_UTILS_H
