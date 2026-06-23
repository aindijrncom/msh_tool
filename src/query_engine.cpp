#ifdef _MSC_VER
#pragma execution_character_set("utf-8")
#endif

#include "query_engine.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <cmath>
#include <limits>
#include <vector>
#include <sstream>

namespace msh {

namespace {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Vec3 subtract(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

double norm(const Vec3& v) {
    return std::sqrt(dot(v, v));
}

bool node_position(const MeshData& data, int node_id, Vec3& p) {
    int idx = node_id - 1;
    if (idx < 0 || static_cast<size_t>(idx) >= data.node_count()) return false;
    p = {data.nodes_x[idx], data.nodes_y[idx], data.nodes_z[idx]};
    return true;
}

bool face_centroid(const MeshData& data, int face_idx, Vec3& centroid) {
    size_t total_faces = data.face_count();
    if (face_idx < 0 || static_cast<size_t>(face_idx) >= total_faces) return false;

    size_t off = data.face_node_offset[face_idx];
    size_t next = (static_cast<size_t>(face_idx + 1) < total_faces)
                      ? data.face_node_offset[face_idx + 1]
                      : data.face_nodes.size();
    int count = 0;
    centroid = {};
    for (size_t j = off; j < next; j++) {
        Vec3 p;
        if (!node_position(data, data.face_nodes[j], p)) continue;
        centroid.x += p.x;
        centroid.y += p.y;
        centroid.z += p.z;
        count++;
    }

    if (count == 0) return false;
    centroid.x /= static_cast<double>(count);
    centroid.y /= static_cast<double>(count);
    centroid.z /= static_cast<double>(count);
    return true;
}

bool face_normal(const MeshData& data, int face_idx, Vec3& normal) {
    size_t total_faces = data.face_count();
    if (face_idx < 0 || static_cast<size_t>(face_idx) >= total_faces) return false;

    size_t off = data.face_node_offset[face_idx];
    size_t next = (static_cast<size_t>(face_idx + 1) < total_faces)
                      ? data.face_node_offset[face_idx + 1]
                      : data.face_nodes.size();
    size_t ncount = next - off;
    if (ncount < 3) return false;

    Vec3 p0, p1, p2;
    if (!node_position(data, data.face_nodes[off], p0) ||
        !node_position(data, data.face_nodes[off + 1], p1) ||
        !node_position(data, data.face_nodes[off + 2], p2)) {
        return false;
    }

    normal = cross(subtract(p1, p0), subtract(p2, p0));

    // Accumulate fan triangles for polygonal faces, matching Validator behavior.
    for (size_t i = 3; i < ncount; i++) {
        Vec3 prev, cur;
        if (!node_position(data, data.face_nodes[off + i - 1], prev) ||
            !node_position(data, data.face_nodes[off + i], cur)) {
            continue;
        }
        Vec3 n = cross(subtract(prev, p0), subtract(cur, p0));
        normal.x += n.x;
        normal.y += n.y;
        normal.z += n.z;
    }

    return norm(normal) > 0.0;
}

struct CentroidFailedCell {
    int cell_id;
    int zone_id;
    double cx, cy, cz;
    int violating_face;
    double violation_distance;
    int face_count;
};

struct CentroidCheckResult {
    int checked = 0;
    int inside = 0;
    int outside = 0;
    int skipped = 0;
    double max_violation = 0.0;
    double tolerance = 0.0;
    std::vector<CentroidFailedCell> failed_cells;
};

CentroidCheckResult compute_centroid_check(const MeshData& data) {
    CentroidCheckResult result;

    int total_cells = static_cast<int>(data.cell_count());
    int total_faces = static_cast<int>(data.face_count());

    std::vector<std::vector<int>> cell_faces(total_cells + 1);
    std::vector<Vec3> cell_centroids(total_cells + 1);
    std::vector<int> cell_face_counts(total_cells + 1, 0);

    Vec3 bmin{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    Vec3 bmax{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};
    for (size_t i = 0; i < data.node_count(); i++) {
        bmin.x = std::min(bmin.x, data.nodes_x[i]);
        bmin.y = std::min(bmin.y, data.nodes_y[i]);
        bmin.z = std::min(bmin.z, data.nodes_z[i]);
        bmax.x = std::max(bmax.x, data.nodes_x[i]);
        bmax.y = std::max(bmax.y, data.nodes_y[i]);
        bmax.z = std::max(bmax.z, data.nodes_z[i]);
    }
    double mesh_scale = data.node_count() > 0 ? norm(subtract(bmax, bmin)) : 1.0;
    if (mesh_scale <= 0.0) mesh_scale = 1.0;
    result.tolerance = mesh_scale * 1e-10;

    for (int f = 0; f < total_faces; f++) {
        Vec3 fc;
        bool has_centroid = face_centroid(data, f, fc);

        int c0 = data.face_c0[f];
        if (c0 > 0 && c0 <= total_cells) {
            cell_faces[c0].push_back(f);
            if (has_centroid) {
                cell_centroids[c0].x += fc.x;
                cell_centroids[c0].y += fc.y;
                cell_centroids[c0].z += fc.z;
                cell_face_counts[c0]++;
            }
        }

        int c1 = data.face_c1[f];
        if (c1 > 0 && c1 <= total_cells) {
            cell_faces[c1].push_back(f);
            if (has_centroid) {
                cell_centroids[c1].x += fc.x;
                cell_centroids[c1].y += fc.y;
                cell_centroids[c1].z += fc.z;
                cell_face_counts[c1]++;
            }
        }
    }

    for (int c = 1; c <= total_cells; c++) {
        if (cell_faces[c].empty() || cell_face_counts[c] == 0) {
            result.skipped++;
            continue;
        }

        cell_centroids[c].x /= static_cast<double>(cell_face_counts[c]);
        cell_centroids[c].y /= static_cast<double>(cell_face_counts[c]);
        cell_centroids[c].z /= static_cast<double>(cell_face_counts[c]);

        result.checked++;
        bool cell_inside = true;
        int worst_face = 0;
        double worst_violation = 0.0;

        for (int f : cell_faces[c]) {
            Vec3 fc, n;
            if (!face_centroid(data, f, fc) || !face_normal(data, f, n)) continue;

            double nmag = norm(n);
            if (nmag <= 0.0) continue;

            double signed_distance = dot(n, subtract(cell_centroids[c], fc)) / nmag;
            double violation = 0.0;
            if (data.face_c0[f] == c) {
                violation = (signed_distance < -result.tolerance) ? -signed_distance : 0.0;
            } else if (data.face_c1[f] == c) {
                violation = (signed_distance > result.tolerance) ? signed_distance : 0.0;
            }

            if (violation > worst_violation) {
                worst_violation = violation;
                worst_face = f + 1;
            }
        }

        if (worst_violation > 0.0) {
            cell_inside = false;
            result.outside++;
            result.max_violation = std::max(result.max_violation, worst_violation);

            CentroidFailedCell cell;
            cell.cell_id = c;
            cell.zone_id = (static_cast<size_t>(c - 1) < data.cell_zones.size()) ? data.cell_zones[c - 1] : 0;
            cell.cx = cell_centroids[c].x;
            cell.cy = cell_centroids[c].y;
            cell.cz = cell_centroids[c].z;
            cell.violating_face = worst_face;
            cell.violation_distance = worst_violation;
            cell.face_count = static_cast<int>(cell_faces[c].size());
            result.failed_cells.push_back(cell);
        } else {
            result.inside++;
        }
    }

    return result;
}

nlohmann::json build_centroid_check_stats(const MeshData& data) {
    auto result = compute_centroid_check(data);
    nlohmann::json j;
    j["method"] = "face-centroid-average + oriented-face-halfspace";
    j["tolerance"] = result.tolerance;
    j["checked"] = result.checked;
    j["inside"] = result.inside;
    j["outside"] = result.outside;
    j["skipped"] = result.skipped;
    j["passed"] = (result.outside == 0);
    j["max_violation_distance"] = result.max_violation;

    nlohmann::json failed_cells = nlohmann::json::array();
    for (size_t i = 0; i < std::min(size_t(20), result.failed_cells.size()); ++i) {
        const auto& c = result.failed_cells[i];
        nlohmann::json item;
        item["cell_id"] = c.cell_id;
        item["zone_id"] = c.zone_id;
        item["centroid"] = {c.cx, c.cy, c.cz};
        item["violating_face"] = c.violating_face;
        item["violation_distance"] = c.violation_distance;
        failed_cells.push_back(item);
    }
    j["failed_cells_sample"] = failed_cells;
    return j;
}

} // namespace

QueryEngine::QueryEngine(const MeshData& data) : data_(data) {}

// ============================================================
// --stats
// ============================================================
nlohmann::json QueryEngine::stats() {
    nlohmann::json j;

    j["dimensions"] = data_.dimensions;
    j["total_nodes"] = data_.node_count();
    j["total_cells"] = data_.cell_count();
    j["total_faces"] = data_.face_count();

    // Cell type distribution
    std::map<int, int> cell_type_dist;
    for (int t : data_.cell_types) cell_type_dist[t]++;
    nlohmann::json ctd = nlohmann::json::object();
    const char* type_names[] = {"mixed", "triangular", "tetrahedral", "quadrilateral",
                                 "hexahedral", "pyramid", "wedge", "polyhedral"};
    for (auto& [k, v] : cell_type_dist) {
        const char* name = (k >= 0 && k <= 7) ? type_names[k] : "unknown";
        ctd[name] = v;
    }
    j["cell_types"] = ctd;

    // Zones
    nlohmann::json zones_json = nlohmann::json::array();
    // Group by zone id
    std::map<int, int> face_zone_counts;
    for (int zid : data_.face_zone_ids) face_zone_counts[zid]++;
    std::map<int, int> cell_zone_counts;
    for (int zid : data_.cell_zones) cell_zone_counts[zid]++;

    std::set<int> all_zone_ids;
    for (auto& [zid, _] : data_.zones) all_zone_ids.insert(zid);
    for (auto& [zid, _] : face_zone_counts) all_zone_ids.insert(zid);
    for (auto& [zid, _] : cell_zone_counts) all_zone_ids.insert(zid);

    for (int zid : all_zone_ids) {
        nlohmann::json z;
        z["id"] = zid;
        auto it = data_.zones.find(zid);
        if (it != data_.zones.end()) {
            z["name"] = it->second.name;
            z["type"] = it->second.type;
        }
        int fc = face_zone_counts[zid];
        int cc = cell_zone_counts[zid];
        z["kind"] = (cc > 0 && fc == 0) ? "cell" : (fc > 0 ? "face" : "unknown");
        if (fc > 0) z["face_count"] = fc;
        if (cc > 0) z["cell_count"] = cc;
        zones_json.push_back(z);
    }
    j["zones"] = zones_json;

    // Tree structures
    j["has_hanging_nodes"] = !data_.face_trees.empty() || !data_.cell_trees.empty();
    j["has_nonconformal"] = !data_.if_parents.empty();
    j["has_periodic"] = !data_.periodic_pairs.empty();

    j["centroid_check"] = build_centroid_check_stats(data_);

    return j;
}

// ============================================================
// --zone <id>
// ============================================================
nlohmann::json QueryEngine::zone(int zone_id) {
    nlohmann::json j;

    // Try both decimal and hex interpretations
    int zid_dec = zone_id;
    int zid_hex = static_cast<int>(std::strtol(std::to_string(zone_id).c_str(), nullptr, 16));

    auto it = data_.zones.find(zid_dec);
    if (it == data_.zones.end()) it = data_.zones.find(zid_hex);

    if (it != data_.zones.end()) {
        j["zone_id"] = it->first;
        j["type"] = it->second.type;
        j["name"] = it->second.name;
    } else {
        j["zone_id"] = zone_id;
        j["type"] = "unknown";
    }

    // Collect faces in this zone
    nlohmann::json faces_json = nlohmann::json::array();
    int total_faces = static_cast<int>(data_.face_count());
    std::map<int, int> face_type_dist;

    for (int f = 0; f < total_faces; f++) {
        if (data_.face_zone_ids[f] == zid_dec || data_.face_zone_ids[f] == zid_hex) {
            nlohmann::json fj;
            fj["id"] = f + 1;

            size_t off = data_.face_node_offset[f];
            size_t next = (f + 1 < total_faces) ? data_.face_node_offset[f + 1] : data_.face_nodes.size();
            nlohmann::json nodes_arr = nlohmann::json::array();
            for (size_t jj = off; jj < next; jj++) {
                nodes_arr.push_back(data_.face_nodes[jj]);
            }
            fj["nodes"] = nodes_arr;
            fj["node_count"] = next - off;

            fj["c0"] = data_.face_c0[f];
            fj["c1"] = data_.face_c1[f];
            faces_json.push_back(fj);

            int ncount = static_cast<int>(next - off);
            face_type_dist[ncount]++;
        }
    }

    j["face_count"] = faces_json.size();
    j["faces"] = faces_json;

    nlohmann::json ftd = nlohmann::json::object();
    for (auto& [n, c] : face_type_dist) {
        std::string label;
        if (n == 2) label = "linear";
        else if (n == 3) label = "triangular";
        else if (n == 4) label = "quadrilateral";
        else label = std::to_string(n) + "-gon";
        ftd[label] = c;
    }
    j["face_types"] = ftd;

    return j;
}

// ============================================================
// --face <id>
// ============================================================
nlohmann::json QueryEngine::face(int face_id) {
    nlohmann::json j;

    int fidx = face_id - 1; // 0-based
    int total_faces = static_cast<int>(data_.face_count());

    if (fidx < 0 || fidx >= total_faces) {
        j["error"] = "Face " + std::to_string(face_id) + " not found (total faces: " +
                     std::to_string(total_faces) + ")";
        return j;
    }

    j["face_id"] = face_id;
    j["zone_id"] = data_.face_zone_ids[fidx];
    j["bc_type"] = data_.face_bc_types[fidx];

    // bc-type name
    const char* bc_name = "unknown";
    switch (data_.face_bc_types[fidx]) {
        case 2:  bc_name = "interior"; break;
        case 3:  bc_name = "wall"; break;
        case 4:  bc_name = "pressure-inlet"; break;
        case 5:  bc_name = "pressure-outlet"; break;
        case 7:  bc_name = "symmetry"; break;
        case 8:  bc_name = "periodic-shadow"; break;
        case 9:  bc_name = "pressure-far-field"; break;
        case 10: bc_name = "velocity-inlet"; break;
        case 12: bc_name = "periodic"; break;
        case 14: bc_name = "fan/porous-jump/radiator"; break;
        case 20: bc_name = "mass-flow-inlet"; break;
        case 24: bc_name = "interface"; break;
        case 31: bc_name = "parent"; break;
        case 36: bc_name = "outflow"; break;
        case 37: bc_name = "axis"; break;
    }
    j["bc_name"] = bc_name;

    // Nodes
    size_t off = data_.face_node_offset[fidx];
    size_t next = (fidx + 1 < total_faces) ? data_.face_node_offset[fidx + 1] : data_.face_nodes.size();

    // Pure ID list
    nlohmann::json node_ids = nlohmann::json::array();
    for (size_t jj = off; jj < next; jj++) node_ids.push_back(data_.face_nodes[jj]);
    j["node_ids"] = node_ids;

    // Detailed: id + coords
    nlohmann::json face_nodes = nlohmann::json::array();
    for (size_t jj = off; jj < next; jj++) {
        int nid = data_.face_nodes[jj];
        nlohmann::json node;
        node["id"] = nid;
        if (nid >= 1 && static_cast<size_t>(nid - 1) < data_.node_count()) {
            node["x"] = data_.nodes_x[nid - 1];
            node["y"] = data_.nodes_y[nid - 1];
            node["z"] = data_.nodes_z[nid - 1];
        }
        face_nodes.push_back(node);
    }
    j["face_nodes"] = face_nodes;
    j["node_count"] = next - off;

    j["c0"] = data_.face_c0[fidx];
    j["c1"] = data_.face_c1[fidx];

    // Right-hand rule check (if face has ≥3 nodes and c0 exists)
    if ((next - off) >= 3 && data_.face_c0[fidx] > 0) {
        int n0 = data_.face_nodes[off] - 1;
        int n1 = data_.face_nodes[off + 1] - 1;
        int n2 = data_.face_nodes[off + 2] - 1;
        if (n0 >= 0 && n1 >= 0 && n2 >= 0 &&
            static_cast<size_t>(n0) < data_.node_count() &&
            static_cast<size_t>(n1) < data_.node_count() &&
            static_cast<size_t>(n2) < data_.node_count()) {

            double e1x = data_.nodes_x[n1] - data_.nodes_x[n0];
            double e1y = data_.nodes_y[n1] - data_.nodes_y[n0];
            double e1z = data_.nodes_z[n1] - data_.nodes_z[n0];
            double e2x = data_.nodes_x[n2] - data_.nodes_x[n0];
            double e2y = data_.nodes_y[n2] - data_.nodes_y[n0];
            double e2z = data_.nodes_z[n2] - data_.nodes_z[n0];

            double nx = e1y * e2z - e1z * e2y;
            double ny = e1z * e2x - e1x * e2z;
            double nz = e1x * e2y - e1y * e2x;

            // Face centroid
            double fcx = 0, fcy = 0, fcz = 0;
            for (size_t jj = off; jj < next; jj++) {
                int ni = data_.face_nodes[jj] - 1;
                fcx += data_.nodes_x[ni];
                fcy += data_.nodes_y[ni];
                fcz += data_.nodes_z[ni];
            }
            fcx /= (next - off); fcy /= (next - off); fcz /= (next - off);

            // c0 centroid (approximate: average of all face centroids for c0)
            int c0 = data_.face_c0[fidx];
            double ccx = 0, ccy = 0, ccz = 0;
            int ccount = 0;
            for (int f2 = 0; f2 < total_faces; f2++) {
                if (data_.face_c0[f2] == c0 || data_.face_c1[f2] == c0) {
                    size_t o2 = data_.face_node_offset[f2];
                    size_t n2 = (f2 + 1 < total_faces) ? data_.face_node_offset[f2 + 1] : data_.face_nodes.size();
                    for (size_t jj = o2; jj < n2; jj++) {
                        int ni = data_.face_nodes[jj] - 1;
                        if (ni >= 0 && static_cast<size_t>(ni) < data_.node_count()) {
                            ccx += data_.nodes_x[ni];
                            ccy += data_.nodes_y[ni];
                            ccz += data_.nodes_z[ni];
                            ccount++;
                        }
                    }
                }
            }
            if (ccount > 0) { ccx /= ccount; ccy /= ccount; ccz /= ccount; }

            double vx = ccx - fcx, vy = ccy - fcy, vz = ccz - fcz;

            // Normalize → cosθ (unitless, scale-invariant)
            double n_mag = std::sqrt(nx*nx + ny*ny + nz*nz);
            double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz);
            if (n_mag > 0) { nx /= n_mag; ny /= n_mag; nz /= n_mag; }
            if (v_mag > 0) { vx /= v_mag; vy /= v_mag; vz /= v_mag; }
            double cos_theta = nx * vx + ny * vy + nz * vz;

            nlohmann::json orient;
            orient["normal"] = {nx, ny, nz};
            orient["cos_theta"] = cos_theta;
            orient["valid"] = (cos_theta > 0.0);
            j["orientation"] = orient;
        }
    }

    return j;
}

// ============================================================
// --cell <id>
// ============================================================
nlohmann::json QueryEngine::cell(int cell_id) {
    nlohmann::json j;

    int cidx = cell_id - 1;
    int total_cells = static_cast<int>(data_.cell_count());

    if (cidx < 0 || cidx >= total_cells) {
        j["error"] = "Cell " + std::to_string(cell_id) + " not found (total cells: " +
                     std::to_string(total_cells) + ")";
        return j;
    }

    j["cell_id"] = cell_id;
    j["zone_id"] = data_.cell_zones[cidx];

    int etype = data_.cell_types[cidx];
    const char* etype_name = "unknown";
    switch (etype) {
        case 0: etype_name = "mixed"; break;
        case 1: etype_name = "triangular"; break;
        case 2: etype_name = "tetrahedral"; break;
        case 3: etype_name = "quadrilateral"; break;
        case 4: etype_name = "hexahedral"; break;
        case 5: etype_name = "pyramid"; break;
        case 6: etype_name = "wedge"; break;
        case 7: etype_name = "polyhedral"; break;
    }
    j["type"] = etype_name;

    // On-demand face scan: find all faces belonging to this cell
    int total_faces = static_cast<int>(data_.face_count());
    nlohmann::json faces_arr = nlohmann::json::array();
    std::set<int> cell_nodes;

    for (int f = 0; f < total_faces; f++) {
        if (data_.face_c0[f] == cell_id || data_.face_c1[f] == cell_id) {
            nlohmann::json fj;
            fj["id"] = f + 1;
            fj["zone_id"] = data_.face_zone_ids[f];
            fj["bc_type"] = data_.face_bc_types[f];

            size_t off = data_.face_node_offset[f];
            size_t next = (f + 1 < total_faces) ? data_.face_node_offset[f + 1] : data_.face_nodes.size();
            nlohmann::json nodes_arr = nlohmann::json::array();
            for (size_t jj = off; jj < next; jj++) {
                nodes_arr.push_back(data_.face_nodes[jj]);
                cell_nodes.insert(data_.face_nodes[jj]);
            }
            fj["nodes"] = nodes_arr;
            fj["node_count"] = next - off;

            faces_arr.push_back(fj);
        }
    }

    j["face_count"] = faces_arr.size();
    j["faces"] = faces_arr;

    nlohmann::json nodes_arr = nlohmann::json::array();
    for (int nid : cell_nodes) nodes_arr.push_back(nid);
    j["nodes"] = nodes_arr;
    j["node_count"] = nodes_arr.size();

    // Volume estimate (centroid-based bounding box diagonal^3 / 20, rough)
    if (!cell_nodes.empty()) {
        double xmin = 1e30, xmax = -1e30, ymin = 1e30, ymax = -1e30, zmin = 1e30, zmax = -1e30;
        for (int nid : cell_nodes) {
            if (nid >= 1 && static_cast<size_t>(nid - 1) < data_.node_count()) {
                double x = data_.nodes_x[nid - 1];
                double y = data_.nodes_y[nid - 1];
                double z = data_.nodes_z[nid - 1];
                if (x < xmin) xmin = x;
                if (x > xmax) xmax = x;
                if (y < ymin) ymin = y;
                if (y > ymax) ymax = y;
                if (z < zmin) zmin = z;
                if (z > zmax) zmax = z;
            }
        }
        double dx = xmax - xmin, dy = ymax - ymin, dz = zmax - zmin;
        j["volume_estimate"] = dx * dy * dz;
        j["bbox"] = {{"xmin", xmin}, {"xmax", xmax}, {"ymin", ymin}, {"ymax", ymax},
                      {"zmin", zmin}, {"zmax", zmax}};
    }

    return j;
}

// ============================================================
// --nodes
// ============================================================
nlohmann::json QueryEngine::nodes() {
    nlohmann::json j;
    j["total"] = data_.node_count();
    nlohmann::json nodes_arr = nlohmann::json::array();
    for (size_t i = 0; i < data_.node_count(); i++) {
        nlohmann::json n;
        n["id"] = i + 1;
        n["x"] = data_.nodes_x[i];
        n["y"] = data_.nodes_y[i];
        n["z"] = data_.nodes_z[i];
        nodes_arr.push_back(n);
    }
    j["nodes"] = nodes_arr;
    return j;
}

// ============================================================
// --validate report
// ============================================================
nlohmann::json QueryEngine::validate_report(const ValidationReport& report) {
    nlohmann::json j;
    j["valid"] = report.valid;
    nlohmann::json errors = nlohmann::json::array();
    nlohmann::json warnings = nlohmann::json::array();
    for (auto& issue : report.issues) {
        nlohmann::json item;
        item["severity"] = (issue.severity == Severity::ERROR) ? "error" : "warning";
        item["message"] = issue.message;
        if (issue.line > 0) item["line"] = issue.line;
        if (issue.face_id > 0) item["face"] = issue.face_id;
        if (issue.cell_id > 0) item["cell"] = issue.cell_id;
        if (issue.zone_id > 0) item["zone"] = issue.zone_id;

        if (issue.severity == Severity::ERROR) errors.push_back(item);
        else warnings.push_back(item);
    }
    j["errors"] = errors;
    j["warnings"] = warnings;
    j["error_count"] = errors.size();
    j["warning_count"] = warnings.size();
    if (report.faces_checked > 0) {
        j["geometry"] = {
            {"faces_checked", report.faces_checked},
            {"passed", report.faces_passed},
            {"failed", report.faces_failed}
        };
    }
    return j;
}

// ============================================================
// CSV export helpers
// ============================================================

std::string csv_escape(const std::string& s) {
    if (s.find(',') != std::string::npos || s.find('"') != std::string::npos || s.find('\n') != std::string::npos) {
        std::string out = "\"";
        for (char c : s) {
            if (c == '"') out += "\"\"";
            else out += c;
        }
        out += "\"";
        return out;
    }
    return s;
}

std::string bc_type_name(int bc) {
    switch (bc) {
        case 2:  return "interior";
        case 3:  return "wall";
        case 4:  return "pressure-inlet";
        case 5:  return "pressure-outlet";
        case 7:  return "symmetry";
        case 8:  return "periodic-shadow";
        case 9:  return "pressure-far-field";
        case 10: return "velocity-inlet";
        case 12: return "periodic";
        case 14: return "fan/porous-jump/radiator";
        case 20: return "mass-flow-inlet";
        case 24: return "interface";
        case 31: return "parent";
        case 36: return "outflow";
        case 37: return "axis";
        default: return "unknown";
    }
}

std::string centroid_recommendation() {
    return "检查该 cell 周围面的绕向，特别是 violating_face 指向的方向";
}

std::string rhr_recommendation(int bc_type, int c1) {
    if (bc_type == 2) {
        return "内部面法向翻转，检查两侧 cell 的绕向一致性";
    }
    if (c1 == 0) {
        return "边界面法向朝外错误，检查 face 节点顺序或 c0/c1 关系";
    }
    return "面法向与 cell 中心向量夹角大于90度，检查面绕向";
}

// ============================================================
// --export-csv <prefix>
// ============================================================
void QueryEngine::export_csv(const std::string& prefix, const ValidationReport& report) const {
    auto centroid_result = compute_centroid_check(data_);

    // 1. summary.csv
    {
        std::string path = prefix + "_summary.csv";
        std::ofstream ofs(path, std::ios::binary);
        if (!ofs.is_open()) {
            std::cerr << "Error: cannot write " << path << std::endl;
            return;
        }

        // UTF-8 BOM for Excel
        ofs.write("\xEF\xBB\xBF", 3);

        int polyhedral_count = 0;
        for (int t : data_.cell_types) {
            if (t == 7) polyhedral_count++;
        }

        size_t error_count = 0;
        size_t warning_count = 0;
        for (const auto& issue : report.issues) {
            if (issue.severity == Severity::ERROR) error_count++;
            else if (issue.severity == Severity::WARNING) warning_count++;
        }

        ofs << "key,value\n";
        ofs << "total_cells," << data_.cell_count() << "\n";
        ofs << "total_faces," << data_.face_count() << "\n";
        ofs << "total_nodes," << data_.node_count() << "\n";
        ofs << "dimensions," << data_.dimensions << "\n";
        ofs << "cell_type_polyhedral," << polyhedral_count << "\n";
        ofs << "valid," << (report.valid ? "true" : "false") << "\n";
        ofs << "centroid_checked," << centroid_result.checked << "\n";
        ofs << "centroid_failed," << centroid_result.outside << "\n";
        ofs << "centroid_passed," << centroid_result.inside << "\n";
        ofs << "centroid_skipped," << centroid_result.skipped << "\n";
        ofs << "rhr_checked," << report.faces_checked << "\n";
        ofs << "rhr_failed," << report.faces_failed << "\n";
        ofs << "rhr_passed," << report.faces_passed << "\n";
        ofs << "max_violation_distance," << std::setprecision(12) << centroid_result.max_violation << "\n";
        ofs << "error_count," << error_count << "\n";
        ofs << "warning_count," << warning_count << "\n";
        ofs.close();
    }

    // 2. centroid_failed.csv
    {
        std::string path = prefix + "_centroid_failed.csv";
        std::ofstream ofs(path, std::ios::binary);
        if (!ofs.is_open()) {
            std::cerr << "Error: cannot write " << path << std::endl;
            return;
        }

        // UTF-8 BOM for Excel
        ofs.write("\xEF\xBB\xBF", 3);

        ofs << "cell_id,zone_id,centroid_x,centroid_y,centroid_z,violating_face,violation_distance,face_count,recommendation\n";
        for (const auto& c : centroid_result.failed_cells) {
            ofs << c.cell_id << ","
               << c.zone_id << ","
               << std::setprecision(12) << c.cx << ","
               << std::setprecision(12) << c.cy << ","
               << std::setprecision(12) << c.cz << ","
               << c.violating_face << ","
               << std::setprecision(12) << c.violation_distance << ","
               << c.face_count << ","
               << csv_escape(centroid_recommendation()) << "\n";
        }
        ofs.close();
    }

    // 3. rhr_failed.csv
    {
        std::string path = prefix + "_rhr_failed.csv";
        std::ofstream ofs(path, std::ios::binary);
        if (!ofs.is_open()) {
            std::cerr << "Error: cannot write " << path << std::endl;
            return;
        }

        // UTF-8 BOM for Excel
        ofs.write("\xEF\xBB\xBF", 3);

        ofs << "face_id,zone_id,bc_type,bc_name,c0,c1,cos_theta,node_count,recommendation\n";
        for (const auto& issue : report.issues) {
            if (issue.severity != Severity::ERROR || issue.face_id <= 0) continue;

            int fidx = issue.face_id - 1;
            if (fidx < 0 || static_cast<size_t>(fidx) >= data_.face_count()) continue;

            int bc = data_.face_bc_types[fidx];
            size_t off = data_.face_node_offset[fidx];
            size_t next = (static_cast<size_t>(fidx + 1) < data_.face_count())
                              ? data_.face_node_offset[fidx + 1]
                              : data_.face_nodes.size();
            int node_count = static_cast<int>(next - off);

            ofs << issue.face_id << ","
               << issue.zone_id << ","
               << bc << ","
               << csv_escape(bc_type_name(bc)) << ","
               << data_.face_c0[fidx] << ","
               << data_.face_c1[fidx] << ","
               << std::setprecision(12) << issue.value << ","
               << node_count << ","
               << csv_escape(rhr_recommendation(bc, data_.face_c1[fidx])) << "\n";
        }
        ofs.close();
    }

    std::cerr << "Exported CSV files:\n"
              << "  " << prefix << "_summary.csv\n"
              << "  " << prefix << "_centroid_failed.csv\n"
              << "  " << prefix << "_rhr_failed.csv" << std::endl;
}

// ============================================================
// Full summary (default mode: validation + stats)
// ============================================================
nlohmann::json QueryEngine::full_summary() {
    nlohmann::json j = stats();
    // Additional summary info
    if (data_.dimensions == 2) {
        j["geometry_type"] = "2D";
    } else {
        j["geometry_type"] = "3D";
    }
    return j;
}

} // namespace msh
