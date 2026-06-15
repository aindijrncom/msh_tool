#pragma once

#include "mesh_data.h"
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>

namespace msh {

// Write MSH mesh to VTK Legacy ASCII format
// Export as PolyData (surface faces only) — simpler, ParaView-stable
inline bool export_vtk(const MeshData& data, const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) return false;

    size_t n_nodes = data.node_count();
    size_t n_faces = data.face_count();

    // Collect all faces (boundary + interior) as polygons
    // Filter to faces with ≥ 3 nodes only
    size_t total_poly_ints = 0;
    for (size_t f = 0; f < n_faces; f++) {
        size_t off = data.face_node_offset[f];
        size_t next = (f + 1 < n_faces)
                          ? data.face_node_offset[f + 1]
                          : data.face_nodes.size();
        int nv = static_cast<int>(next - off);
        if (nv >= 3) {
            total_poly_ints += 1 + nv; // npts + node ids
        }
    }

    // --- Write VTK header ---
    out << "# vtk DataFile Version 3.0\n";
    out << "Fluent MSH export\n";
    out << "ASCII\n";
    out << "DATASET POLYDATA\n\n";

    // --- POINTS ---
    out << "POINTS " << n_nodes << " double\n";
    for (size_t i = 0; i < n_nodes; i++) {
        out << data.nodes_x[i] << " " << data.nodes_y[i] << " " << data.nodes_z[i] << "\n";
    }

    // --- POLYGONS ---
    size_t n_polys = 0;
    for (size_t f = 0; f < n_faces; f++) {
        size_t off = data.face_node_offset[f];
        size_t next = (f + 1 < n_faces)
                          ? data.face_node_offset[f + 1]
                          : data.face_nodes.size();
        if (static_cast<int>(next - off) >= 3) n_polys++;
    }

    out << "\nPOLYGONS " << n_polys << " " << total_poly_ints << "\n";
    for (size_t f = 0; f < n_faces; f++) {
        size_t off = data.face_node_offset[f];
        size_t next = (f + 1 < n_faces)
                          ? data.face_node_offset[f + 1]
                          : data.face_nodes.size();
        int nv = static_cast<int>(next - off);
        if (nv < 3) continue;
        out << nv;
        for (size_t j = off; j < next; j++) {
            out << " " << (data.face_nodes[j] - 1); // 1→0 based
        }
        out << "\n";
    }

    // --- CELL_DATA: face metadata ---
    out << "\nCELL_DATA " << n_polys << "\n";

    // face_id (1-based MSH face index)
    out << "SCALARS face_id int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (size_t f = 0; f < n_faces; f++) {
        size_t off = data.face_node_offset[f];
        size_t next = (f + 1 < n_faces) ? data.face_node_offset[f + 1] : data.face_nodes.size();
        if (static_cast<int>(next - off) < 3) continue;
        out << (f + 1) << "\n"; // 1-based face ID
    }

    // c0 (adjacent cell)
    out << "SCALARS c0 int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (size_t f = 0; f < n_faces; f++) {
        size_t off = data.face_node_offset[f];
        size_t next = (f + 1 < n_faces) ? data.face_node_offset[f + 1] : data.face_nodes.size();
        if (static_cast<int>(next - off) < 3) continue;
        out << data.face_c0[f] << "\n";
    }

    // c1 (adjacent cell)
    out << "SCALARS c1 int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (size_t f = 0; f < n_faces; f++) {
        size_t off = data.face_node_offset[f];
        size_t next = (f + 1 < n_faces) ? data.face_node_offset[f + 1] : data.face_nodes.size();
        if (static_cast<int>(next - off) < 3) continue;
        out << data.face_c1[f] << "\n";
    }

    // bc_type
    out << "SCALARS bc_type int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (size_t f = 0; f < n_faces; f++) {
        size_t off = data.face_node_offset[f];
        size_t next = (f + 1 < n_faces) ? data.face_node_offset[f + 1] : data.face_nodes.size();
        if (static_cast<int>(next - off) < 3) continue;
        out << data.face_bc_types[f] << "\n";
    }

    // zone_id
    out << "SCALARS zone_id int 1\n";
    out << "LOOKUP_TABLE default\n";
    for (size_t f = 0; f < n_faces; f++) {
        size_t off = data.face_node_offset[f];
        size_t next = (f + 1 < n_faces) ? data.face_node_offset[f + 1] : data.face_nodes.size();
        if (static_cast<int>(next - off) < 3) continue;
        out << data.face_zone_ids[f] << "\n";
    }

    return true;
}

// ============================================================
// VTK XML UnstructuredGrid (.vtu) — full polyhedral volume cells
// ============================================================
inline bool export_vtk_volume(const MeshData& data, const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) return false;

    size_t n_nodes = data.node_count();
    size_t n_cells = data.cell_count();
    size_t n_faces = data.face_count();

    // --- Build cell→faces mapping ---
    std::vector<std::vector<std::pair<int, bool>>> cell_faces(n_cells + 1);
    for (size_t f = 0; f < n_faces; f++) {
        int c0 = data.face_c0[f];
        int c1 = data.face_c1[f];
        if (c0 > 0 && static_cast<size_t>(c0) <= n_cells)
            cell_faces[c0].push_back({static_cast<int>(f), false});
        if (c1 > 0 && static_cast<size_t>(c1) <= n_cells)
            cell_faces[c1].push_back({static_cast<int>(f), true});
    }

    auto face_nverts = [&](int fidx) -> int {
        size_t off = data.face_node_offset[fidx];
        size_t next = (fidx + 1 < static_cast<int>(n_faces))
                          ? data.face_node_offset[fidx + 1]
                          : data.face_nodes.size();
        return static_cast<int>(next - off);
    };

    // --- Build VTK arrays ---
    std::vector<int64_t> connectivity;
    std::vector<int64_t> offsets;
    std::vector<int64_t> faces_stream;
    std::vector<int64_t> faceoffsets;

    for (size_t c = 1; c <= n_cells; c++) {
        auto& cfaces = cell_faces[c];

        // --- faces array entry for this cell ---
        // Format: nCellFaces, (nVerts_f0, v0..vn), (nVerts_f1, v0..vn), ...
        int n_cell_faces = static_cast<int>(cfaces.size());
        faces_stream.push_back(n_cell_faces);

        for (auto& [fidx, flip] : cfaces) {
            int nv = face_nverts(fidx);
            size_t off = data.face_node_offset[fidx];
            faces_stream.push_back(nv);
            if (flip) {
                for (int j = nv - 1; j >= 0; j--) {
                    int64_t vid = data.face_nodes[off + j] - 1; // 0-based
                    faces_stream.push_back(vid);
                    connectivity.push_back(vid);
                }
            } else {
                for (int j = 0; j < nv; j++) {
                    int64_t vid = data.face_nodes[off + j] - 1; // 0-based
                    faces_stream.push_back(vid);
                    connectivity.push_back(vid);
                }
            }
        }

        // offsets: cumulative connectivity count after this cell
        offsets.push_back(static_cast<int64_t>(connectivity.size()));

        // faceoffsets: cumulative faces stream count after this cell
        faceoffsets.push_back(static_cast<int64_t>(faces_stream.size()));
    }

    // --- Write XML ---
    out << "<?xml version=\"1.0\"?>\n";
    out << "<VTKFile type=\"UnstructuredGrid\" version=\"1.0\" byte_order=\"LittleEndian\" header_type=\"UInt64\">\n";
    out << "  <UnstructuredGrid>\n";
    out << "    <Piece NumberOfPoints=\"" << n_nodes << "\" NumberOfCells=\"" << n_cells << "\">\n";

    // Points
    out << "      <Points>\n";
    out << "        <DataArray type=\"Float64\" Name=\"Points\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (size_t i = 0; i < n_nodes; i++) {
        out << "          " << data.nodes_x[i] << " " << data.nodes_y[i] << " " << data.nodes_z[i] << "\n";
    }
    out << "        </DataArray>\n";
    out << "      </Points>\n";

    // Cells
    out << "      <Cells>\n";
    out << "        <DataArray type=\"Int64\" Name=\"connectivity\" format=\"ascii\">\n";
    for (size_t i = 0; i < connectivity.size(); i++) {
        if (i > 0 && i % 20 == 0) out << "\n";
        out << "          " << connectivity[i] << "\n";
    }
    out << "        </DataArray>\n";

    out << "        <DataArray type=\"Int64\" Name=\"offsets\" format=\"ascii\">\n";
    for (size_t i = 0; i < offsets.size(); i++) {
        out << "          " << offsets[i] << "\n";
    }
    out << "        </DataArray>\n";

    out << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n";
    for (size_t i = 0; i < n_cells; i++) {
        out << "          42\n"; // VTK_POLYHEDRON
    }
    out << "        </DataArray>\n";

    out << "        <DataArray type=\"Int64\" Name=\"faces\" format=\"ascii\">\n";
    for (size_t i = 0; i < faces_stream.size(); i++) {
        if (i > 0 && i % 20 == 0) out << "\n";
        out << "          " << faces_stream[i] << "\n";
    }
    out << "        </DataArray>\n";

    out << "        <DataArray type=\"Int64\" Name=\"faceoffsets\" format=\"ascii\">\n";
    for (size_t i = 0; i < faceoffsets.size(); i++) {
        out << "          " << faceoffsets[i] << "\n";
    }
    out << "        </DataArray>\n";
    out << "      </Cells>\n";

    // Cell data: cell_id + zone_id
    out << "      <CellData Scalars=\"cell_id\">\n";
    out << "        <DataArray type=\"Int32\" Name=\"cell_id\" format=\"ascii\">\n";
    for (size_t c = 1; c <= n_cells; c++) {
        out << "          " << c << "\n"; // 1-based MSH cell ID
    }
    out << "        </DataArray>\n";
    out << "        <DataArray type=\"Int32\" Name=\"zone_id\" format=\"ascii\">\n";
    for (size_t c = 0; c < n_cells; c++) {
        out << "          " << data.cell_zones[c] << "\n";
    }
    out << "        </DataArray>\n";
    out << "      </CellData>\n";

    out << "    </Piece>\n";
    out << "  </UnstructuredGrid>\n";
    out << "</VTKFile>\n";

    return true;
}

} // namespace msh
