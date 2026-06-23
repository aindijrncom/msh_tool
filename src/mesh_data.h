#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <cstdint>
#include <cstddef>

namespace msh {

// ============================================================
// Node storage: SoA layout for cache efficiency
// ============================================================
struct NodeZone {
    int zone_id;      // zone this node block belongs to
    size_t first;     // first node index (1-based, converted to 0-based in storage)
    size_t count;     // number of nodes in this zone
};

struct MeshData {
    // --- Nodes ---
    std::vector<double> nodes_x;
    std::vector<double> nodes_y;
    std::vector<double> nodes_z;
    std::vector<NodeZone> node_zones;

    size_t total_nodes_declared = 0;  // from declaration section (zone-id=0)
    int dimensions = 0;               // 2 or 3, from section 2

    // --- Faces (CSR format for nodes) ---
    std::vector<int> face_zone_ids;
    std::vector<int> face_bc_types;
    std::vector<int> face_c0;
    std::vector<int> face_c1;
    std::vector<size_t> face_node_offset;  // CSR: start position in face_nodes
    std::vector<int> face_nodes;           // CSR: concatenated node indices

    size_t total_faces_declared = 0;

    // --- Cells ---
    std::vector<int> cell_types;   // element-type for each cell
    std::vector<int> cell_zones;   // zone for each cell

    size_t total_cells_declared = 0;

    // --- Zones ---
    struct ZoneInfo {
        std::string name;
        std::string type;   // fluid, solid, wall, interior, etc.
        int domain_id = 0;
    };
    // Key: zone-id (decimal). Grid section zone-ids are converted from hex.
    std::unordered_map<int, ZoneInfo> zones;

    // --- Periodic Shadow Faces ---
    struct PeriodicPair {
        int periodic_zone;
        int shadow_zone;
        std::vector<std::pair<int, int>> face_pairs;  // (periodic_face, shadow_face)
    };
    std::vector<PeriodicPair> periodic_pairs;

    // --- Hanging Node Trees ---
    struct FaceTreeEntry {
        int parent_face_id;
        int parent_zone_id;
        int child_zone_id;
        std::vector<int> child_face_ids;
    };
    std::vector<FaceTreeEntry> face_trees;

    struct CellTreeEntry {
        int parent_cell_id;
        int parent_zone_id;
        int child_zone_id;
        std::vector<int> child_cell_ids;
    };
    std::vector<CellTreeEntry> cell_trees;

    // --- Non-conformal Interface Parents ---
    struct IFParentEntry {
        int child_face_id;
        std::vector<int> parent_face_ids;
    };
    std::vector<IFParentEntry> if_parents;

    // --- Utility ---
    // Get total node count (across all zones)
    size_t node_count() const { return nodes_x.size(); }
    size_t face_count() const { return face_zone_ids.size(); }
    size_t cell_count() const { return cell_types.size(); }
};

// ============================================================
// Validation types
// ============================================================
enum class Severity { ERROR, WARNING };

struct ValidationIssue {
    Severity severity;
    std::string message;
    int line = 0;          // 0 = not location-specific
    int face_id = 0;
    int cell_id = 0;
    int zone_id = 0;
    double value = 0.0;    // numeric context, e.g. cos_theta for RHR errors
};

struct ValidationReport {
    bool valid = true;
    std::vector<ValidationIssue> issues;
    // Statistics
    size_t faces_checked = 0;
    size_t faces_passed = 0;
    size_t faces_failed = 0;

    bool has_errors() const {
        for (auto& i : issues) if (i.severity == Severity::ERROR) return true;
        return false;
    }
};

} // namespace msh
