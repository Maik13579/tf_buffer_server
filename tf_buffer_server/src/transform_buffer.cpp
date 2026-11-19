// Copyright 2026 Maik Knof
// SPDX-License-Identifier: Apache-2.0
#include "transform_buffer.hpp"

namespace tf_buffer_server
{
namespace
{
constexpr const char * authority = "tf_buffer_server";
}

void TransformBuffer::insert(const tf2_msgs::msg::TFMessage & message, bool is_static)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  for (const auto & transform : message.transforms) {
    if (!setTransform(transform, authority, is_static)) {
      continue;
    }
    // TF accepts a leading slash on incoming frame IDs and normalizes it internally.
    const auto & child = transform.child_frame_id;
    const auto key = child.front() == '/' ? child.substr(1) : child;
    if (is_static) {
      static_transforms_[key] = transform;
    } else {
      static_transforms_.erase(key);
    }
  }
}

void TransformBuffer::clear()
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  tf2::BufferCore::clear();
  // Lyrical clears static caches too. Restore synchronously, without DDS rediscovery.
  for (const auto & entry : static_transforms_) {
    setTransform(entry.second, authority, true);
  }
}
}  // namespace tf_buffer_server
