#pragma once

#include <algorithm>
#include <vector>

namespace flow {

inline float median(std::vector<float> values) {
  std::sort(values.begin(), values.end());
  const auto n = values.size();
  return (values[(n - 1) / 2] + values[n / 2]) * 0.5f;
}

} // namespace flow
