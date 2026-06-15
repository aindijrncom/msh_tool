#pragma once

#include "mesh_data.h"
#include <nlohmann/json.hpp>

namespace msh {

// ============================================================
// QueryEngine: all query modes → JSON output
// ============================================================
class QueryEngine {
public:
    explicit QueryEngine(const MeshData& data);

    nlohmann::json stats();
    nlohmann::json zone(int zone_id);
    nlohmann::json face(int face_id);
    nlohmann::json cell(int cell_id);
    nlohmann::json nodes();
    nlohmann::json validate_report(const ValidationReport& report);
    nlohmann::json full_summary();

private:
    const MeshData& data_;
};

} // namespace msh
