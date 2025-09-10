#include <tesseract_open3d/utils.h>

// STD
#include <cstdint>
#include <vector>
#include <string>

// Open3d
#include <open3d/core/Device.h>
#include <open3d/core/Tensor.h>
#include <open3d/core/TensorFunction.h>
#include <open3d/t/geometry/PointCloud.h>

namespace tesseract_o3d
{
namespace detail
{
// Concatenate two attribute tensors along dim 0, padding with zeros if one side
// is missing. `set_attr` is a setter on `output` (e.g., SetPointColors).
template <typename HasFnOut, typename GetFnOut, typename SetFnOut, typename HasFnIn, typename GetFnIn>
void concatAttrWithPad(open3d::t::geometry::PointCloud& output,
                       const open3d::t::geometry::PointCloud& input_on_dev,
                       int64_t n_out_old,
                       int64_t n_in,
                       HasFnOut has_out,
                       GetFnOut get_out,
                       SetFnOut set_out,
                       HasFnIn has_in,
                       GetFnIn get_in)
{
  const open3d::core::Device dev = output.GetDevice();

  if (has_out(output))
  {
    auto a = get_out(output);  // existing (rows = n_out_old)
    open3d::core::Tensor b;
    if (has_in(input_on_dev))
    {
      b = get_in(input_on_dev);  // rows = n_in
      if (b.GetDtype() != a.GetDtype())
        b = b.To(dev, a.GetDtype());
    }
    else
    {
      // pad zeros for new rows
      auto shape = a.GetShape();
      shape[0] = n_in;
      b = open3d::core::Tensor::Zeros(shape, a.GetDtype(), dev);
    }
    set_out(output, open3d::core::Concatenate({ a, b }, 0));
  }
  else if (has_in(input_on_dev))
  {
    // Add this attribute to output: zeros for old rows + input data
    auto b = get_in(input_on_dev);
    auto shape_old = b.GetShape();
    shape_old[0] = n_out_old;
    auto zeros_old = open3d::core::Tensor::Zeros(shape_old, b.GetDtype(), dev);
    auto cat = open3d::core::Concatenate({ zeros_old, b }, 0);
    set_out(output, cat);
  }
}

}  // namespace detail

void combinePointCloud(open3d::t::geometry::PointCloud& output,
                       const open3d::t::geometry::PointCloud& input,
                       const std::vector<std::string>& custom_attributes)
{
  using open3d::core::Tensor;
  using open3d::t::geometry::PointCloud;

  if (input.IsEmpty())
    return;

  // Ensure both on the same device (prefer output’s, or use input’s if empty)
  const auto target_dev = output.IsEmpty() ? input.GetDevice() : output.GetDevice();
  PointCloud in_dev = input.To(target_dev);

  // If output is empty, just clone input (copies all existing attrs)
  if (output.IsEmpty())
  {
    output = in_dev.Clone();
    return;
  }

  // Concatenate positions
  const Tensor p_out = output.GetPointPositions();  // [N0, 3]
  const Tensor p_in = in_dev.GetPointPositions();   // [N1, 3]
  const int64_t n_out_old = p_out.GetShape(0);
  const int64_t n_in = p_in.GetShape(0);

  output.SetPointPositions(open3d::core::Concatenate({ p_out, p_in }, 0));

  // Colors (if present on either side)
  detail::concatAttrWithPad(
      output,
      in_dev,
      n_out_old,
      n_in,
      [](const PointCloud& pc) { return pc.HasPointColors(); },
      [](PointCloud& pc) { return pc.GetPointColors(); },
      [](PointCloud& pc, const Tensor& t) { pc.SetPointColors(t); },
      [](const PointCloud& pc) { return pc.HasPointColors(); },
      [](const PointCloud& pc) { return pc.GetPointColors(); });

  // Normals (if present on either side)
  detail::concatAttrWithPad(
      output,
      in_dev,
      n_out_old,
      n_in,
      [](const PointCloud& pc) { return pc.HasPointNormals(); },
      [](PointCloud& pc) { return pc.GetPointNormals(); },
      [](PointCloud& pc, const Tensor& t) { pc.SetPointNormals(t); },
      [](const PointCloud& pc) { return pc.HasPointNormals(); },
      [](const PointCloud& pc) { return pc.GetPointNormals(); });

  // --- Example for a custom attribute (uncomment & adapt) --------------------
  for (const auto& attribute : custom_attributes)
  {
    detail::concatAttrWithPad(
        output,
        in_dev,
        n_out_old,
        n_in,
        [attribute](const PointCloud& pc) { return pc.HasPointAttr(attribute); },
        [attribute](PointCloud& pc) { return pc.GetPointAttr(attribute); },
        [attribute](PointCloud& pc, const Tensor& t) { pc.SetPointAttr(attribute, t); },
        [attribute](const PointCloud& pc) { return pc.HasPointAttr(attribute); },
        [attribute](const PointCloud& pc) { return pc.GetPointAttr(attribute); });
  }
}

}  // namespace tesseract_o3d
