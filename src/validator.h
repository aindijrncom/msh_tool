#pragma once

#include "mesh_data.h"

namespace msh {

// ============================================================
// Validator: 4-layer validation
//   Layer 1: Syntax (collected during parsing)
//   Layer 2: Structure (count consistency, index overlap, zone refs)
//   Layer 3: Topology (bound checks, interior/boundary rules, tree validity)
//   Layer 4: Geometry (right-hand rule orientation)
// ============================================================
class Validator {
public:
    explicit Validator(const MeshData& data);

    // Run all validation layers
    ValidationReport validate(bool skip_geometry = false);

    // Individual layers (can be called separately)
    void validate_structure(ValidationReport& report);
    void validate_topology(ValidationReport& report);
    void validate_geometry(ValidationReport& report);

private:
    const MeshData& data_;

    // Geometry helpers
    void compute_centroid(int cell_id, double& cx, double& cy, double& cz) const;
    bool check_right_hand_rule(size_t face_idx, double& dot_product, double& nx, double& ny, double& nz) const;
};

} // namespace msh
