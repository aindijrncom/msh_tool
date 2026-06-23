#include "lexer.h"
#include "parser.h"
#include "validator.h"
#include "query_engine.h"
#include "json_colorizer.h"
#include "vtk_export.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cstdlib>

#ifdef _WIN32
#include <windows.h>
#undef ERROR  // Windows macro clashes with Severity::ERROR
#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif
#endif

// Helper: output JSON with optional color
void output_json(const nlohmann::json& j, bool color) {
    if (color) {
        std::cout << msh::colorize_json(j);
    } else {
        std::cout << j.dump(2) << std::endl;
    }
}

void print_help(const char* prog) {
    std::cout << R"(USAGE: )" << prog << R"( <file.msh> [options]

A diagnostic tool for Fluent MSH mesh files.
Reads formatted (.msh) files, validates integrity and geometry,
and outputs structured JSON to stdout.

OPTIONS:
  Query modes (mutually exclusive):
    -s, --stats          Statistics summary only
    -z, --zone <id>      Face details for a specific zone
    -f, --face <id>      Connectivity & orientation for a specific face
    -c, --cell <id>      Reconstruct cell geometry from faces
    -n, --nodes          Dump all node coordinates
    -v, --validate       Validation report only (no summary)
    -e, --error-faces    List error face IDs with reasons

  Validation control:
    -V, --no-validate    Skip all validation checks
    -G, --no-geometry    Skip right-hand-rule geometry check

  General:
    -o, --export-vtk [prefix]  Export [prefix].vtk + .vtu (default: derived from .msh)
    --export-csv [prefix]      Export diagnostic CSV files (default: derived from .msh)
    -S, --scale <factor>       Multiply all coordinates (default: 1000, m→mm)
    -C, --no-color             Disable ANSI color output
    -h, --help                 Print this help text

EXIT CODES:
  0   Validation passed (or --no-validate used)
  1   Validation errors found
  2   File not found / cannot open
  3   Parse error (malformed MSH)
)";
}

int main(int argc, char* argv[]) {
    // Set console to UTF-8 + enable ANSI escape codes on Windows
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    // Enable ANSI virtual terminal processing for color output
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut != INVALID_HANDLE_VALUE) {
        DWORD mode = 0;
        if (GetConsoleMode(hOut, &mode)) {
            mode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
            SetConsoleMode(hOut, mode);
        }
    }
#endif

    // --- Parse CLI ---
    std::string filepath;
    std::string query_mode;  // "stats", "zone", "face", "cell", "nodes", "validate"
    int query_id = 0;        // zone/face/cell id
    bool no_validate = false;
    bool no_geometry = false;
    bool use_color = true;  // color on by default
    double scale = 1000.0;  // m → mm by default
    std::string vtk_prefix;      // --export-vtk base name
    std::string csv_prefix;      // --export-csv base name

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            print_help(argv[0]);
            return 0;
        }
        else if (arg == "--no-validate" || arg == "-V") {
            no_validate = true;
        }
        else if (arg == "--no-geometry" || arg == "-G") {
            no_geometry = true;
        }
        else if (arg == "--no-color" || arg == "-C") {
            use_color = false;
        }
        else if (arg == "--export-vtk" || arg == "-o") {
            // Optional argument: if next arg doesn't start with '-', use it as prefix
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                vtk_prefix = argv[++i];
            } else {
                vtk_prefix = "AUTO"; // auto-derive from msh path
            }
        }
        else if (arg == "--export-csv") {
            // Optional argument: if next arg doesn't start with '-', use it as prefix
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                csv_prefix = argv[++i];
            } else {
                csv_prefix = "AUTO"; // auto-derive from msh path
            }
        }
        else if (arg == "--scale" || arg == "-S") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --scale requires a number" << std::endl;
                return 3;
            }
            scale = std::atof(argv[++i]);
            if (scale <= 0.0) {
                std::cerr << "Error: --scale must be positive" << std::endl;
                return 3;
            }
        }
        else if (arg == "--stats" || arg == "-s") {
            query_mode = "stats";
        }
        else if (arg == "--nodes" || arg == "-n") {
            query_mode = "nodes";
        }
        else if (arg == "--validate" || arg == "-v") {
            query_mode = "validate";
        }
        else if (arg == "--error-faces" || arg == "-e") {
            query_mode = "error_faces";
        }
        else if (arg == "--zone" || arg == "-z") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --zone requires an argument" << std::endl;
                return 3;
            }
            query_mode = "zone";
            query_id = std::atoi(argv[++i]);
        }
        else if (arg == "--face" || arg == "-f") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --face requires an argument" << std::endl;
                return 3;
            }
            query_mode = "face";
            query_id = std::atoi(argv[++i]);
        }
        else if (arg == "--cell" || arg == "-c") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --cell requires an argument" << std::endl;
                return 3;
            }
            query_mode = "cell";
            query_id = std::atoi(argv[++i]);
        }
        else if (arg[0] == '-') {
            std::cerr << "Error: Unknown option: " << arg << std::endl;
            std::cerr << "Use --help for usage information." << std::endl;
            return 3;
        }
        else {
            if (filepath.empty()) {
                filepath = arg;
            } else {
                std::cerr << "Error: Multiple input files specified" << std::endl;
                return 3;
            }
        }
    }

    if (filepath.empty()) {
        std::cerr << "Error: No input file specified." << std::endl;
        std::cerr << "Use --help for usage information." << std::endl;
        return 3;
    }

    // --- Open file ---
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file: " << filepath << std::endl;
        return 2;
    }

    // --- Parse ---
    msh::Lexer lexer(file);
    msh::Parser parser(lexer);

    msh::MeshData mesh;
    try {
        mesh = parser.parse();
    } catch (const std::exception& e) {
        nlohmann::json err;
        err["error"] = "Parse failed";
        err["message"] = e.what();
        output_json(err, use_color);
        return 3;
    }

    // --- Scale coordinates (default: m → mm, factor 1000) ---
    if (scale != 1.0) {
        for (auto& x : mesh.nodes_x) x *= scale;
        for (auto& y : mesh.nodes_y) y *= scale;
        for (auto& z : mesh.nodes_z) z *= scale;
    }

    // --- Export VTK (optional, generates both .vtk and .vtu) ---
    if (!vtk_prefix.empty()) {
        // Auto-derive prefix from msh path if not explicitly given
        if (vtk_prefix == "AUTO") {
            std::string base = filepath;
            // Strip .msh extension
            if (base.size() > 4) {
                auto ext = base.substr(base.size() - 4);
                if (ext == ".msh" || ext == ".MSH") {
                    base = base.substr(0, base.size() - 4);
                }
            }
            vtk_prefix = base;
        }

        std::string faces_file = vtk_prefix + ".vtk";
        std::string volume_file = vtk_prefix + ".vtu";

        if (!msh::export_vtk(mesh, faces_file)) {
            std::cerr << "Error: Failed to write: " << faces_file << std::endl;
            return 3;
        }
        if (!msh::export_vtk_volume(mesh, volume_file)) {
            std::cerr << "Error: Failed to write: " << volume_file << std::endl;
            return 3;
        }

        std::cerr << "Exported:\n"
                  << "  " << faces_file << "  (faces, PolyData)\n"
                  << "  " << volume_file << "  (cells, UnstructuredGrid)\n"
                  << "  nodes: " << mesh.node_count()
                  << "  cells: " << mesh.cell_count()
                  << "  faces: " << mesh.face_count()
                  << "  zones: " << mesh.zones.size() << std::endl;

        // If only export (no other query mode), exit cleanly
        if (query_mode.empty()) return 0;
    }

    // Collect syntax issues from parser
    msh::ValidationReport syntax_report;
    for (auto& issue : parser.syntax_issues()) {
        syntax_report.issues.push_back(issue);
    }
    bool has_syntax_errors = syntax_report.has_errors();

    // --- Validate ---
    msh::ValidationReport validation_report;
    if (!no_validate) {
        msh::Validator validator(mesh);
        validation_report = validator.validate(no_geometry);

        // Prepend syntax issues
        for (auto& issue : syntax_report.issues) {
            validation_report.issues.insert(validation_report.issues.begin(), issue);
        }
        validation_report.valid = validation_report.valid && !has_syntax_errors;
    }

    // --- Query ---
    msh::QueryEngine qe(mesh);

    // --- Export CSV (optional) ---
    if (!csv_prefix.empty()) {
        if (csv_prefix == "AUTO") {
            std::string base = filepath;
            if (base.size() > 4) {
                auto ext = base.substr(base.size() - 4);
                if (ext == ".msh" || ext == ".MSH") {
                    base = base.substr(0, base.size() - 4);
                }
            }
            csv_prefix = base;
        }
        if (!no_validate) {
            qe.export_csv(csv_prefix, validation_report);
        } else {
            std::cerr << "Warning: --export-csv requires validation; use without -V" << std::endl;
        }
    }

    if (query_mode.empty()) {
        // Default: validation + summary
        nlohmann::json output;
        if (!no_validate) {
            msh::QueryEngine qe2(mesh);
            output["validation"] = qe2.validate_report(validation_report);
        }
        output["mesh"] = qe.full_summary();
        output_json(output, use_color);
    }
    else if (query_mode == "stats") {
        nlohmann::json output;
        if (!no_validate) {
            output["validation"] = qe.validate_report(validation_report);
        }
        output["mesh"] = qe.stats();
        output_json(output, use_color);
    }
    else if (query_mode == "validate") {
        output_json(qe.validate_report(validation_report), use_color);
    }
    else if (query_mode == "error_faces") {
        nlohmann::json err_faces = nlohmann::json::array();
        for (auto& issue : validation_report.issues) {
            if (issue.severity == msh::Severity::ERROR && issue.face_id > 0) {
                nlohmann::json item;
                item["face"] = issue.face_id;
                item["zone"] = issue.zone_id;
                item["reason"] = issue.message;
                err_faces.push_back(item);
            }
        }
        nlohmann::json out;
        out["error_faces"] = err_faces;
        out["count"] = err_faces.size();
        output_json(out, use_color);
    }
    else if (query_mode == "zone") {
        output_json(qe.zone(query_id), use_color);
    }
    else if (query_mode == "face") {
        output_json(qe.face(query_id), use_color);
    }
    else if (query_mode == "cell") {
        output_json(qe.cell(query_id), use_color);
    }
    else if (query_mode == "nodes") {
        output_json(qe.nodes(), use_color);
    }

    // --- Exit code ---
    if (!no_validate && validation_report.has_errors()) {
        return 1;
    }

    return 0;
}
