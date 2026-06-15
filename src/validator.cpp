#include "validator.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace msh {

Validator::Validator(const MeshData& data) : data_(data) {}

// ============================================================
// Main validate entry
// ============================================================
ValidationReport Validator::validate(bool skip_geometry) {
    ValidationReport report;
    validate_structure(report);
    // Run topology regardless — topology errors are local and don't
    // invalidate geometry checks on other faces.
    validate_topology(report);
    if (!skip_geometry) {
        validate_geometry(report);
    }
    report.valid = !report.has_errors();
    return report;
}

// ============================================================
// Layer 2: Structure validation
// ============================================================
void Validator::validate_structure(ValidationReport& report) {
    // --- Check required sections exist ---
    if (data_.total_nodes_declared == 0) {
        report.issues.push_back({Severity::ERROR, "Missing Nodes declaration section (zone-id=0)"});
    }
    if (data_.total_faces_declared == 0) {
        report.issues.push_back({Severity::ERROR, "Missing Faces declaration section (zone-id=0)"});
    }
    if (data_.total_cells_declared == 0 && data_.cell_types.empty()) {
        // Surface mesh is OK - cells may be absent
    }
    if (data_.zones.empty()) {
        report.issues.push_back({Severity::WARNING, "No Zone sections found"});
    }

    // --- Node count consistency ---
    if (data_.node_count() != data_.total_nodes_declared) {
        report.issues.push_back({Severity::ERROR,
            "Node count mismatch: declared " + std::to_string(data_.total_nodes_declared) +
            ", actual " + std::to_string(data_.node_count())});
    }

    // --- Face count consistency ---
    if (data_.face_count() != data_.total_faces_declared) {
        report.issues.push_back({Severity::ERROR,
            "Face count mismatch: declared " + std::to_string(data_.total_faces_declared) +
            ", actual " + std::to_string(data_.face_count())});
    }

    // --- Cell count consistency ---
    if (data_.cell_count() != data_.total_cells_declared && data_.total_cells_declared > 0) {
        report.issues.push_back({Severity::ERROR,
            "Cell count mismatch: declared " + std::to_string(data_.total_cells_declared) +
            ", actual " + std::to_string(data_.cell_count())});
    }

    // --- Zone reference consistency ---
    std::set<int> grid_zone_ids;
    // Collect zone-ids from nodes
    for (auto& nz : data_.node_zones) {
        grid_zone_ids.insert(nz.zone_id);
    }
    // From faces
    for (auto zid : data_.face_zone_ids) {
        grid_zone_ids.insert(zid);
    }
    // From cells
    for (auto zid : data_.cell_zones) {
        grid_zone_ids.insert(zid);
    }
    // Check against Zone sections
    for (int gzid : grid_zone_ids) {
        if (data_.zones.find(gzid) == data_.zones.end()) {
            report.issues.push_back({Severity::WARNING,
                "Grid zone " + std::to_string(gzid) + " has no corresponding Zone section"});
        }
    }

    // --- Dimension consistency ---
    if (data_.dimensions != 2 && data_.dimensions != 3 && data_.dimensions != 0) {
        report.issues.push_back({Severity::ERROR,
            "Invalid dimensions: " + std::to_string(data_.dimensions) + " (expected 2 or 3)"});
    }
}

// ============================================================
// Layer 3: Topology validation
// ============================================================
void Validator::validate_topology(ValidationReport& report) {
    int total_nodes = static_cast<int>(data_.node_count());
    int total_cells = static_cast<int>(data_.cell_count());
    int total_faces = static_cast<int>(data_.face_count());

    // --- Check face node & cell references ---
    for (int f = 0; f < total_faces; f++) {
        int zid = data_.face_zone_ids[f];
        int bc = data_.face_bc_types[f];
        int c0 = data_.face_c0[f];
        int c1 = data_.face_c1[f];

        // Node bounds
        size_t off = data_.face_node_offset[f];
        size_t next_off = (f + 1 < total_faces) ? data_.face_node_offset[f + 1] : data_.face_nodes.size();
        for (size_t j = off; j < next_off; j++) {
            int nid = data_.face_nodes[j];
            if (nid < 1 || nid > total_nodes) {
                report.issues.push_back({Severity::ERROR,
                    "Face " + std::to_string(f + 1) + " in zone " + std::to_string(zid) +
                    ": node index " + std::to_string(nid) + " out of range [1, " +
                    std::to_string(total_nodes) + "]"});
                break; // one error per face is enough
            }
        }

        // Cell bounds
        if (c0 < 0 || c0 > total_cells) {
            report.issues.push_back({Severity::ERROR,
                "Face " + std::to_string(f + 1) + ": c0=" + std::to_string(c0) +
                " out of range [0, " + std::to_string(total_cells) + "]"});
        }
        if (c1 < 0 || c1 > total_cells) {
            report.issues.push_back({Severity::ERROR,
                "Face " + std::to_string(f + 1) + ": c1=" + std::to_string(c1) +
                " out of range [0, " + std::to_string(total_cells) + "]"});
        }

        // Interior faces (bc=2): both c0 and c1 must be non-zero
        if (bc == 2 && (c0 == 0 || c1 == 0)) {
            report.issues.push_back({Severity::ERROR,
                "Face " + std::to_string(f + 1) + " (interior): c0=" + std::to_string(c0) +
                " c1=" + std::to_string(c1) + ": interior face must reference two cells",
                0, f + 1, 0, zid});
        }

        // Boundary faces (bc in {3,4,5,7,8,9,10,12,14,20,24,36,37}): c1 should be 0
        if (bc != 2 && bc != 0 && c1 != 0) {
            report.issues.push_back({Severity::WARNING,
                "Face " + std::to_string(f + 1) + " (bc-type=" + std::to_string(bc) +
                "): c1=" + std::to_string(c1) + ": boundary face typically has c1=0"});
        }
    }

    // --- Face Tree validation ---
    for (auto& ft : data_.face_trees) {
        if (ft.parent_face_id < 1 || ft.parent_face_id > total_faces) {
            report.issues.push_back({Severity::ERROR,
                "Face Tree: parent face " + std::to_string(ft.parent_face_id) + " out of range"});
        }
        for (int cid : ft.child_face_ids) {
            if (cid < 1 || cid > total_faces) {
                report.issues.push_back({Severity::ERROR,
                    "Face Tree: child face " + std::to_string(cid) + " out of range"});
            }
        }
        if (ft.child_face_ids.empty()) {
            report.issues.push_back({Severity::ERROR,
                "Face Tree: parent face " + std::to_string(ft.parent_face_id) + " has no children"});
        }
    }

    // --- Cell Tree validation ---
    for (auto& ct : data_.cell_trees) {
        if (ct.parent_cell_id < 1 || ct.parent_cell_id > total_cells) {
            report.issues.push_back({Severity::ERROR,
                "Cell Tree: parent cell " + std::to_string(ct.parent_cell_id) + " out of range"});
        }
        for (int cid : ct.child_cell_ids) {
            if (cid < 1 || cid > total_cells) {
                report.issues.push_back({Severity::ERROR,
                    "Cell Tree: child cell " + std::to_string(cid) + " out of range"});
            }
        }
    }

    // --- Interface Face Parents validation ---
    for (auto& ip : data_.if_parents) {
        if (ip.child_face_id < 1 || ip.child_face_id > total_faces) {
            report.issues.push_back({Severity::ERROR,
                "Interface Face Parents: child face " + std::to_string(ip.child_face_id) + " out of range"});
        }
        for (int pid : ip.parent_face_ids) {
            if (pid < 1 || pid > total_faces) {
                report.issues.push_back({Severity::ERROR,
                    "Interface Face Parents: parent face " + std::to_string(pid) + " out of range"});
            }
        }
    }

    // --- Periodic Shadow Face validation ---
    for (auto& pp : data_.periodic_pairs) {
        for (auto& [pf, sf] : pp.face_pairs) {
            if (pf < 1 || pf > total_faces) {
                report.issues.push_back({Severity::ERROR,
                    "Periodic: face " + std::to_string(pf) + " out of range"});
            }
            if (sf < 1 || sf > total_faces) {
                report.issues.push_back({Severity::ERROR,
                    "Periodic: shadow face " + std::to_string(sf) + " out of range"});
            }
        }
    }
}

// ============================================================
// Layer 4: Geometry validation (right-hand rule)
// ============================================================
void Validator::compute_centroid(int cell_id, double& cx, double& cy, double& cz) const {
    cx = cy = cz = 0.0;
    int count = 0;
    int total_faces = static_cast<int>(data_.face_count());

    // Average of face centroids for faces belonging to this cell
    for (int f = 0; f < total_faces; f++) {
        if (data_.face_c0[f] == cell_id || data_.face_c1[f] == cell_id) {
            size_t off = data_.face_node_offset[f];
            size_t next_off = (f + 1 < total_faces) ? data_.face_node_offset[f + 1] : data_.face_nodes.size();
            double fx = 0, fy = 0, fz = 0;
            int ncount = 0;
            for (size_t j = off; j < next_off; j++) {
                int nid = data_.face_nodes[j] - 1; // 0-based
                if (nid >= 0 && static_cast<size_t>(nid) < data_.node_count()) {
                    fx += data_.nodes_x[nid];
                    fy += data_.nodes_y[nid];
                    fz += data_.nodes_z[nid];
                    ncount++;
                }
            }
            if (ncount > 0) {
                cx += fx / ncount;
                cy += fy / ncount;
                cz += fz / ncount;
                count++;
            }
        }
    }
    if (count > 0) {
        cx /= count;
        cy /= count;
        cz /= count;
    }
}

bool Validator::check_right_hand_rule(size_t face_idx, double& dot_product,
                                       double& nx, double& ny, double& nz) const {
    size_t off = data_.face_node_offset[face_idx];
    size_t next_off = (face_idx + 1 < data_.face_count())
                          ? data_.face_node_offset[face_idx + 1]
                          : data_.face_nodes.size();
    size_t ncount = next_off - off;

    if (ncount < 3) return true; // 2-node faces (edges) can't check orientation

    // Get first 3 nodes (0-based indices)
    int n0 = data_.face_nodes[off] - 1;
    int n1 = data_.face_nodes[off + 1] - 1;
    int n2 = data_.face_nodes[off + 2] - 1;

    if (n0 < 0 || n1 < 0 || n2 < 0) return true;
    size_t un0 = static_cast<size_t>(n0);
    size_t un1 = static_cast<size_t>(n1);
    size_t un2 = static_cast<size_t>(n2);
    if (un0 >= data_.node_count() || un1 >= data_.node_count() || un2 >= data_.node_count())
        return true;

    // Edge vectors
    double e1x = data_.nodes_x[un1] - data_.nodes_x[un0];
    double e1y = data_.nodes_y[un1] - data_.nodes_y[un0];
    double e1z = data_.nodes_z[un1] - data_.nodes_z[un0];

    double e2x = data_.nodes_x[un2] - data_.nodes_x[un0];
    double e2y = data_.nodes_y[un2] - data_.nodes_y[un0];
    double e2z = data_.nodes_z[un2] - data_.nodes_z[un0];

    // Cross product: N = e1 × e2
    nx = e1y * e2z - e1z * e2y;
    ny = e1z * e2x - e1x * e2z;
    nz = e1x * e2y - e1y * e2x;

    // For polygonal faces, average with additional triples
    for (size_t i = 3; i < ncount; i++) {
        int ni = data_.face_nodes[off + i] - 1;
        if (ni < 0 || static_cast<size_t>(ni) >= data_.node_count()) continue;
        size_t uni = static_cast<size_t>(ni);
        int prev = data_.face_nodes[off + i - 1] - 1;
        if (prev < 0 || static_cast<size_t>(prev) >= data_.node_count()) continue;
        size_t uprev = static_cast<size_t>(prev);

        double d1x = data_.nodes_x[uprev] - data_.nodes_x[un0];
        double d1y = data_.nodes_y[uprev] - data_.nodes_y[un0];
        double d1z = data_.nodes_z[uprev] - data_.nodes_z[un0];
        double d2x = data_.nodes_x[uni] - data_.nodes_x[un0];
        double d2y = data_.nodes_y[uni] - data_.nodes_y[un0];
        double d2z = data_.nodes_z[uni] - data_.nodes_z[un0];

        nx += d1y * d2z - d1z * d2y;
        ny += d1z * d2x - d1x * d2z;
        nz += d1x * d2y - d1y * d2x;
    }

    // Face centroid
    double fcx = 0, fcy = 0, fcz = 0;
    for (size_t j = off; j < next_off; j++) {
        int nid = data_.face_nodes[j] - 1;
        if (nid >= 0 && static_cast<size_t>(nid) < data_.node_count()) {
            fcx += data_.nodes_x[nid];
            fcy += data_.nodes_y[nid];
            fcz += data_.nodes_z[nid];
        }
    }
    fcx /= static_cast<double>(ncount);
    fcy /= static_cast<double>(ncount);
    fcz /= static_cast<double>(ncount);

    // c0 centroid
    int c0 = data_.face_c0[face_idx];
    double ccx, ccy, ccz;
    compute_centroid(c0, ccx, ccy, ccz);

    // Vector from face centroid to cell centroid
    double vx = ccx - fcx;
    double vy = ccy - fcy;
    double vz = ccz - fcz;

    dot_product = nx * vx + ny * vy + nz * vz;
    return dot_product > 0.0;
}

void Validator::validate_geometry(ValidationReport& report) {
    // Pre-compute cell centroids cache to avoid recomputation
    // (compute_centroid is called per-face and is O(F) each time)
    // We'll check each face and cache results
    int total_cells = static_cast<int>(data_.cell_count());
    std::vector<double> cell_cx(total_cells + 1, 0.0);
    std::vector<double> cell_cy(total_cells + 1, 0.0);
    std::vector<double> cell_cz(total_cells + 1, 0.0);
    std::vector<int> cell_face_count(total_cells + 1, 0);
    std::vector<double> cell_fx(total_cells + 1, 0.0); // accumulated face centroid x
    std::vector<double> cell_fy(total_cells + 1, 0.0);
    std::vector<double> cell_fz(total_cells + 1, 0.0);

    int total_faces = static_cast<int>(data_.face_count());

    // Pass 1: accumulate face centroids to cell centroid sums
    for (int f = 0; f < total_faces; f++) {
        size_t off = data_.face_node_offset[f];
        size_t next_off = (f + 1 < total_faces) ? data_.face_node_offset[f + 1] : data_.face_nodes.size();
        double fx = 0, fy = 0, fz = 0;
        int ncount = 0;
        for (size_t j = off; j < next_off; j++) {
            int nid = data_.face_nodes[j] - 1;
            if (nid >= 0 && static_cast<size_t>(nid) < data_.node_count()) {
                fx += data_.nodes_x[nid];
                fy += data_.nodes_y[nid];
                fz += data_.nodes_z[nid];
                ncount++;
            }
        }
        if (ncount > 0) {
            fx /= ncount; fy /= ncount; fz /= ncount;
            int c0 = data_.face_c0[f];
            if (c0 > 0 && c0 <= total_cells) {
                cell_fx[c0] += fx; cell_fy[c0] += fy; cell_fz[c0] += fz;
                cell_face_count[c0]++;
            }
            int c1 = data_.face_c1[f];
            if (c1 > 0 && c1 <= total_cells) {
                cell_fx[c1] += fx; cell_fy[c1] += fy; cell_fz[c1] += fz;
                cell_face_count[c1]++;
            }
        }
    }

    // Compute cell centroids
    for (int c = 1; c <= total_cells; c++) {
        if (cell_face_count[c] > 0) {
            cell_cx[c] = cell_fx[c] / cell_face_count[c];
            cell_cy[c] = cell_fy[c] / cell_face_count[c];
            cell_cz[c] = cell_fz[c] / cell_face_count[c];
        }
    }

    // Pass 2: check right-hand rule for each face
    report.faces_checked = total_faces;
    for (int f = 0; f < total_faces; f++) {
        int c0 = data_.face_c0[f];
        if (c0 <= 0 || c0 > total_cells) continue; // boundary or invalid

        size_t off = data_.face_node_offset[f];
        size_t next_off = (f + 1 < total_faces) ? data_.face_node_offset[f + 1] : data_.face_nodes.size();
        size_t ncount = next_off - off;
        if (ncount < 3) continue; // skip 2-node "faces" (edges)

        // Get first 3 nodes
        int n0 = data_.face_nodes[off] - 1;
        int n1 = data_.face_nodes[off + 1] - 1;
        int n2 = data_.face_nodes[off + 2] - 1;
        if (n0 < 0 || n1 < 0 || n2 < 0) continue;
        size_t un0 = static_cast<size_t>(n0), un1 = static_cast<size_t>(n1), un2 = static_cast<size_t>(n2);
        if (un0 >= data_.node_count() || un1 >= data_.node_count() || un2 >= data_.node_count()) continue;

        // Edge vectors
        double e1x = data_.nodes_x[un1] - data_.nodes_x[un0];
        double e1y = data_.nodes_y[un1] - data_.nodes_y[un0];
        double e1z = data_.nodes_z[un1] - data_.nodes_z[un0];
        double e2x = data_.nodes_x[un2] - data_.nodes_x[un0];
        double e2y = data_.nodes_y[un2] - data_.nodes_y[un0];
        double e2z = data_.nodes_z[un2] - data_.nodes_z[un0];

        // Normal
        double nx = e1y * e2z - e1z * e2y;
        double ny = e1z * e2x - e1x * e2z;
        double nz = e1x * e2y - e1y * e2x;

        // Add contributions from additional node triples (polygonal faces)
        for (size_t i = 3; i < ncount; i++) {
            int ni = data_.face_nodes[off + i] - 1;
            int prev = data_.face_nodes[off + i - 1] - 1;
            if (ni < 0 || prev < 0) continue;
            size_t uni = static_cast<size_t>(ni), uprev = static_cast<size_t>(prev);
            if (uni >= data_.node_count() || uprev >= data_.node_count()) continue;

            double d1x = data_.nodes_x[uprev] - data_.nodes_x[un0];
            double d1y = data_.nodes_y[uprev] - data_.nodes_y[un0];
            double d1z = data_.nodes_z[uprev] - data_.nodes_z[un0];
            double d2x = data_.nodes_x[uni] - data_.nodes_x[un0];
            double d2y = data_.nodes_y[uni] - data_.nodes_y[un0];
            double d2z = data_.nodes_z[uni] - data_.nodes_z[un0];

            nx += d1y * d2z - d1z * d2y;
            ny += d1z * d2x - d1x * d2z;
            nz += d1x * d2y - d1y * d2x;
        }

        // Face centroid
        double fcx = 0, fcy = 0, fcz = 0;
        for (size_t j = off; j < next_off; j++) {
            int nid = data_.face_nodes[j] - 1;
            if (nid >= 0 && static_cast<size_t>(nid) < data_.node_count()) {
                fcx += data_.nodes_x[nid];
                fcy += data_.nodes_y[nid];
                fcz += data_.nodes_z[nid];
            }
        }
        fcx /= static_cast<double>(ncount);
        fcy /= static_cast<double>(ncount);
        fcz /= static_cast<double>(ncount);

        // Cell centroid (pre-computed)
        double ccx = cell_cx[c0], ccy = cell_cy[c0], ccz = cell_cz[c0];

        // Direction vector
        double vx = ccx - fcx, vy = ccy - fcy, vz = ccz - fcz;

        // Normalize both vectors → dot = cosθ (unitless, scale-invariant)
        double n_mag = std::sqrt(nx*nx + ny*ny + nz*nz);
        double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz);
        if (n_mag > 0) { nx /= n_mag; ny /= n_mag; nz /= n_mag; }
        if (v_mag > 0) { vx /= v_mag; vy /= v_mag; vz /= v_mag; }
        double cos_theta = nx * vx + ny * vy + nz * vz;
        // cosθ ≈  1: correct orientation
        // cosθ ≈  0: degenerate (face ⊥ cell-centroid vector → grid quality)
        // cosθ ≈ -1: truly flipped (mesh export bug)

        if (cos_theta <= 0.0) {
            report.issues.push_back({Severity::ERROR,
                "Face " + std::to_string(f + 1) + " (zone " + std::to_string(data_.face_zone_ids[f]) +
                "): right-hand rule violation, cos(theta) = " + std::to_string(cos_theta) +
                " (expected > 0)",
                0, f + 1, 0, data_.face_zone_ids[f]});
            report.faces_failed++;
        } else {
            report.faces_passed++;
        }
    }
}

} // namespace msh
