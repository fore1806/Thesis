#pragma once
// =============================================================================
//  association_utils.hpp
//  Shared utilities: colour palette, OBB CropBox, point-cloud builder.
// =============================================================================

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lidar_camera_association
{

// ---------------------------------------------------------------------------
// Colour palette  (object_id % N → RGB in [0,1])
// ---------------------------------------------------------------------------

struct Colour { float r, g, b; };

inline Colour colour_for_id(int id)
{
  static constexpr std::array<Colour, 8> kPalette{{
    {0.94f, 0.27f, 0.27f},  // red
    {0.27f, 0.67f, 0.94f},  // sky-blue
    {0.27f, 0.94f, 0.54f},  // mint
    {0.94f, 0.75f, 0.27f},  // amber
    {0.75f, 0.27f, 0.94f},  // violet
    {0.94f, 0.50f, 0.27f},  // orange
    {0.27f, 0.94f, 0.94f},  // cyan
    {0.94f, 0.27f, 0.75f},  // pink
  }};
  int idx = ((id % 8) + 8) % 8;          // safe modulo for negative ids
  return kPalette[static_cast<std::size_t>(idx)];
}

// Pack r,g,b floats [0,1] into a single float (RViz2 RGB32 convention).
inline float pack_rgb(float r, float g, float b)
{
  uint32_t ri = static_cast<uint32_t>(r * 255.f);
  uint32_t gi = static_cast<uint32_t>(g * 255.f);
  uint32_t bi = static_cast<uint32_t>(b * 255.f);
  uint32_t packed = (ri << 16) | (gi << 8) | bi;
  float out{};
  std::memcpy(&out, &packed, sizeof(float));
  return out;
}

// ---------------------------------------------------------------------------
// CropBox filter – pure Eigen, no PCL required
//
// corners : 8×3 matrix, rows ordered as ZED SDK:
//            0-1-2-3  bottom face  (CCW when viewed from below)
//            4-5-6-7  top face
//
// The three OBB axes are derived from edges 0→1, 0→3, 0→4.
// Returns true for each point that lies strictly inside [0, edge_len]+margin.
// ---------------------------------------------------------------------------

inline std::vector<bool> cropbox_filter(
  const Eigen::MatrixX3f & pts,        // N×3
  const Eigen::Matrix<float, 8, 3> & corners,
  float margin = 0.0f)
{
  const Eigen::Vector3f origin = corners.row(0);
  const Eigen::Vector3f ax = corners.row(1) - corners.row(0);
  const Eigen::Vector3f ay = corners.row(3) - corners.row(0);
  const Eigen::Vector3f az = corners.row(4) - corners.row(0);

  float lx = ax.norm(), ly = ay.norm(), lz = az.norm();

  std::vector<bool> mask(static_cast<std::size_t>(pts.rows()), false);

  if (lx < 1e-6f || ly < 1e-6f || lz < 1e-6f) return mask;

  const Eigen::Vector3f ux = ax / lx;
  const Eigen::Vector3f uy = ay / ly;
  const Eigen::Vector3f uz = az / lz;

  for (int i = 0; i < pts.rows(); ++i) {
    const Eigen::Vector3f d = pts.row(i).transpose() - origin;
    float px = d.dot(ux);
    float py = d.dot(uy);
    float pz = d.dot(uz);
    mask[static_cast<std::size_t>(i)] =
      (px >= -margin) && (px <= lx + margin) &&
      (py >= -margin) && (py <= ly + margin) &&
      (pz >= -margin) && (pz <= lz + margin);
  }
  return mask;
}

}  // namespace lidar_camera_association
