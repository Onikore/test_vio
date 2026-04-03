#include "vio_slam/backend/marginalization.h"

namespace vio_slam {
namespace backend {

Marginalization::Marginalization() {}

void Marginalization::marginalizeOut(int variable_index) {
  // Placeholder - actual implementation would use Schur complement
  marginalized_indices_.push_back(variable_index);
}

}  // namespace backend
}  // namespace vio_slam
