#include "parser.h"
#include <stdexcept>
#include <cstdlib>
#include <cstring>

namespace msh {

Parser::Parser(Lexer& lexer) : lexer_(lexer) {}

// ============================================================
// Hex/Dec parsing
// ============================================================
int64_t Parser::parse_int(const Token& t, bool hex) {
    if (hex) {
        return std::strtoll(t.text.c_str(), nullptr, 16);
    }
    return t.int_value; // already parsed as dec in Lexer
}

int64_t Parser::parse_hex_int(const Token& t) {
    return std::strtoll(t.text.c_str(), nullptr, 16);
}

// ============================================================
// Token helpers
// ============================================================
Token Parser::expect(TokenType type, const std::string& context) {
    auto t = lexer_.next();
    if (t.type != type) {
        std::string msg = "Expected " + std::to_string(static_cast<int>(type)) +
                          " in " + context + " at line " + std::to_string(t.line);
        throw std::runtime_error(msg);
    }
    current_line_ = t.line;
    return t;
}

Token Parser::expect_int(const std::string& context) {
    return expect(TokenType::INTEGER, context);
}

Token Parser::expect_float(const std::string& context) {
    auto t = lexer_.next();
    if (t.type != TokenType::INTEGER && t.type != TokenType::FLOAT) {
        throw std::runtime_error("Expected number in " + context + " at line " + std::to_string(t.line));
    }
    current_line_ = t.line;
    // If it looks like an integer but is actually a float, convert
    if (t.type == TokenType::INTEGER) {
        t.float_value = static_cast<double>(t.int_value);
    }
    return t;
}

void Parser::add_syntax_error(const std::string& msg) {
    syntax_issues_.push_back({Severity::ERROR, msg, static_cast<int>(current_line_)});
}

void Parser::add_syntax_warning(const std::string& msg) {
    syntax_issues_.push_back({Severity::WARNING, msg, static_cast<int>(current_line_)});
}

// ============================================================
// Section body reading (for mixed types)
// ============================================================
void Parser::read_section_body_tokens(std::vector<Token>& out) {
    // Read tokens until we hit the closing RPAREN at depth 0
    int depth = 0;
    while (true) {
        auto t = lexer_.next();
        if (t.type == TokenType::END_OF_FILE) {
            throw std::runtime_error("Unexpected EOF while reading section body");
        }
        if (t.type == TokenType::LPAREN) {
            depth++;
            continue; // skip sub-parens for mixed body reading
        }
        if (t.type == TokenType::RPAREN) {
            if (depth > 0) { depth--; continue; }
            break; // end of section
        }
        if (depth == 0) {
            out.push_back(t);
        }
    }
}

int Parser::read_paren_depth() {
    // Read one token and if it's LPAREN, return 1; else it should be part of body
    auto t = lexer_.peek();
    if (t.type == TokenType::LPAREN) {
        lexer_.next(); // consume
        return 1;
    }
    return 0;
}

// ============================================================
// Skip unknown section (match parens)
// ============================================================
void Parser::skip_section() {
    // Section header already consumed (the section index and opening paren)
    // We need to skip to the matching closing paren
    int depth = 1; // we're already inside the section's outer parens
    while (depth > 0) {
        auto t = lexer_.next();
        if (t.type == TokenType::END_OF_FILE) {
            add_syntax_error("Unexpected EOF while skipping unknown section");
            return;
        }
        if (t.type == TokenType::LPAREN) depth++;
        if (t.type == TokenType::RPAREN) depth--;
    }
}

// ============================================================
// Section 0: Comment
// ============================================================
void Parser::parse_comment() {
    // (0 "comment text")
    auto t = lexer_.next();
    if (t.type == TokenType::STRING) {
        // Just consume and ignore
    }
    // Consume closing paren
    expect(TokenType::RPAREN, "comment");
}

// ============================================================
// Section 1: Header
// ============================================================
void Parser::parse_header() {
    // (1 "program info")
    auto t = lexer_.next();
    if (t.type == TokenType::STRING) {
        // Just consume and ignore
    }
    expect(TokenType::RPAREN, "header");
}

// ============================================================
// Section 2: Dimensions
// ============================================================
void Parser::parse_dimensions() {
    // (2 ND)
    auto t = expect_int("dimensions");
    data_.dimensions = static_cast<int>(parse_int(t, false)); // ND is decimal
    expect(TokenType::RPAREN, "dimensions");
}

// ============================================================
// Section 10: Nodes
// ============================================================
void Parser::parse_nodes() {
    // (10 (zone-id first-index last-index type ND)(coords...))
    expect(TokenType::LPAREN, "nodes header");

    auto zone_id_t = expect_int("nodes zone-id");
    int zone_id = static_cast<int>(parse_hex_int(zone_id_t));
    auto first_t = expect_int("nodes first-index");
    int first = static_cast<int>(parse_hex_int(first_t));
    auto last_t = expect_int("nodes last-index");
    int last = static_cast<int>(parse_hex_int(last_t));
    auto type_t = expect_int("nodes type");
    (void)parse_hex_int(type_t); // consume and discard type

    int nd = data_.dimensions;
    // ND might be present in header
    auto peek = lexer_.peek();
    if (peek.type == TokenType::INTEGER) {
        auto nd_t = expect_int("nodes ND");
        nd = static_cast<int>(parse_hex_int(nd_t));
    }

    expect(TokenType::RPAREN, "nodes header close");

    if (zone_id == 0) {
        // Declaration section: record total count
        data_.total_nodes_declared = static_cast<size_t>(last);
        // No coordinates follow - the next token should be RPAREN
        expect(TokenType::RPAREN, "nodes declaration close");
    } else {
        // Data section with coordinates
        expect(TokenType::LPAREN, "nodes data open");
        int count = last - first + 1;

        NodeZone nz;
        nz.zone_id = zone_id;
        nz.first = data_.nodes_x.size();
        nz.count = static_cast<size_t>(count);
        data_.node_zones.push_back(nz);

        for (int i = 0; i < count; i++) {
            if (nd == 2) {
                auto tx = expect_float("node x");
                auto ty = expect_float("node y");
                data_.nodes_x.push_back(tx.float_value);
                data_.nodes_y.push_back(ty.float_value);
                data_.nodes_z.push_back(0.0);
            } else {
                auto tx = expect_float("node x");
                auto ty = expect_float("node y");
                auto tz = expect_float("node z");
                data_.nodes_x.push_back(tx.float_value);
                data_.nodes_y.push_back(ty.float_value);
                data_.nodes_z.push_back(tz.float_value);
            }
        }

        expect(TokenType::RPAREN, "nodes data close");
        expect(TokenType::RPAREN, "nodes section close");
    }
}

// ============================================================
// Section 12: Cells
// ============================================================
void Parser::parse_cells() {
    // (12 (zone-id first-index last-index type element-type))
    expect(TokenType::LPAREN, "cells header");

    auto zid_t = expect_int("cells zone-id");
    int zone_id = static_cast<int>(parse_hex_int(zid_t));
    auto first_t = expect_int("cells first-index");
    int first = static_cast<int>(parse_hex_int(first_t));
    auto last_t = expect_int("cells last-index");
    int last = static_cast<int>(parse_hex_int(last_t));
    auto type_t = expect_int("cells type");
    (void)parse_hex_int(type_t); // consume and discard type

    int etype = -1;
    auto peek = lexer_.peek();
    if (peek.type == TokenType::INTEGER) {
        auto etype_t = expect_int("cells element-type");
        etype = static_cast<int>(parse_hex_int(etype_t));
    }

    expect(TokenType::RPAREN, "cells header close");

    if (zone_id == 0) {
        // Declaration section
        data_.total_cells_declared = static_cast<size_t>(last);
        expect(TokenType::RPAREN, "cells declaration close");
    } else {
        int count = last - first + 1;

        if (etype == 0) {
            // Mixed type: body lists each cell's element-type
            expect(TokenType::LPAREN, "cells mixed body open");
            std::vector<Token> body;
            read_section_body_tokens(body);

            for (size_t i = 0; i < body.size() && static_cast<int>(i) < count; i++) {
                int cell_etype = static_cast<int>(parse_hex_int(body[i]));
                data_.cell_types.push_back(cell_etype);
                data_.cell_zones.push_back(zone_id);
            }
            expect(TokenType::RPAREN, "cells mixed section close");
        } else if (etype > 0) {
            // Non-mixed: no body, all cells have the same element-type
            for (int i = 0; i < count; i++) {
                data_.cell_types.push_back(etype);
                data_.cell_zones.push_back(zone_id);
            }
            expect(TokenType::RPAREN, "cells section close");
        } else {
            // etype == -1: no element-type in header, skip
            // Check if there's a body
            auto p = lexer_.peek();
            if (p.type == TokenType::LPAREN) {
                lexer_.next(); // consume LPAREN
                std::vector<Token> body;
                read_section_body_tokens(body);
                for (size_t i = 0; i < body.size() && static_cast<int>(i) < count; i++) {
                    int cell_etype = static_cast<int>(parse_hex_int(body[i]));
                    data_.cell_types.push_back(cell_etype);
                    data_.cell_zones.push_back(zone_id);
                }
            } else {
                expect(TokenType::RPAREN, "cells section close");
            }
        }
    }
}

// ============================================================
// Section 13: Faces
// ============================================================
void Parser::parse_faces() {
    // (13 (zone-id first-index last-index bc-type face-type))
    expect(TokenType::LPAREN, "faces header");

    auto zid_t = expect_int("faces zone-id");
    int zone_id = static_cast<int>(parse_hex_int(zid_t));
    auto first_t = expect_int("faces first-index");
    int first = static_cast<int>(parse_hex_int(first_t));
    auto last_t = expect_int("faces last-index");
    int last = static_cast<int>(parse_hex_int(last_t));

    int bc_type = 0;
    int face_type = 0;
    auto peek = lexer_.peek();
    if (peek.type == TokenType::INTEGER) {
        auto bc_t = expect_int("faces bc-type");
        bc_type = static_cast<int>(parse_hex_int(bc_t));
    }
    peek = lexer_.peek();
    if (peek.type == TokenType::INTEGER) {
        auto ft_t = expect_int("faces face-type");
        face_type = static_cast<int>(parse_hex_int(ft_t));
    }

    expect(TokenType::RPAREN, "faces header close");

    if (zone_id == 0) {
        // Declaration section
        data_.total_faces_declared = static_cast<size_t>(last);
        expect(TokenType::RPAREN, "faces declaration close");
    } else {
        int count = last - first + 1;

        // Read body
        expect(TokenType::LPAREN, "faces body open");
        std::vector<Token> body_tokens;
        read_section_body_tokens(body_tokens);

        size_t idx = 0;
        for (int i = 0; i < count && idx < body_tokens.size(); i++) {
            int nnodes;
            if (face_type == 0 || face_type == 5) {
                // Mixed or polygonal: first token is node count
                nnodes = static_cast<int>(parse_hex_int(body_tokens[idx]));
                idx++;
            } else {
                // Fixed node count based on face_type:
                // 2=linear(2), 3=triangular(3), 4=quadrilateral(4)
                nnodes = (face_type == 2) ? 2 : (face_type == 3) ? 3 : 4;
            }

            data_.face_node_offset.push_back(data_.face_nodes.size());

            // Read nodes
            for (int j = 0; j < nnodes && idx < body_tokens.size(); j++) {
                int nid = static_cast<int>(parse_hex_int(body_tokens[idx]));
                data_.face_nodes.push_back(nid);
                idx++;
            }

            // Read c0, c1
            int c0 = 0, c1 = 0;
            if (idx < body_tokens.size()) {
                c0 = static_cast<int>(parse_hex_int(body_tokens[idx]));
                idx++;
            }
            if (idx < body_tokens.size()) {
                c1 = static_cast<int>(parse_hex_int(body_tokens[idx]));
                idx++;
            }

            data_.face_zone_ids.push_back(zone_id);
            data_.face_bc_types.push_back(bc_type);
            data_.face_c0.push_back(c0);
            data_.face_c1.push_back(c1);
        }

        expect(TokenType::RPAREN, "faces section close");
    }
}

// ============================================================
// Section 18: Periodic Shadow Faces
// ============================================================
void Parser::parse_periodic_shadow_faces() {
    expect(TokenType::LPAREN, "periodic header");

    auto first_t = expect_int("periodic first-index");
    int first = static_cast<int>(parse_hex_int(first_t));
    auto last_t = expect_int("periodic last-index");
    int last = static_cast<int>(parse_hex_int(last_t));
    auto pzone_t = expect_int("periodic periodic-zone");
    int pzone = static_cast<int>(parse_hex_int(pzone_t));
    auto szone_t = expect_int("periodic shadow-zone");
    int szone = static_cast<int>(parse_hex_int(szone_t));

    expect(TokenType::RPAREN, "periodic header close");

    int count = last - first + 1;

    expect(TokenType::LPAREN, "periodic body open");
    std::vector<Token> body;
    read_section_body_tokens(body);

    MeshData::PeriodicPair pp;
    pp.periodic_zone = pzone;
    pp.shadow_zone = szone;

    for (int i = 0; i < count * 2 && static_cast<size_t>(i + 1) < body.size(); i += 2) {
        int pf = static_cast<int>(parse_hex_int(body[i]));
        int sf = static_cast<int>(parse_hex_int(body[i + 1]));
        pp.face_pairs.push_back({pf, sf});
    }

    data_.periodic_pairs.push_back(pp);
    expect(TokenType::RPAREN, "periodic section close");
}

// ============================================================
// Section 39 / 45: Zone
// ============================================================
void Parser::parse_zone(int /*section_index*/) {
    in_grid_section_ = false;

    expect(TokenType::LPAREN, "zone header");

    auto zid_t = expect_int("zone zone-id");
    int zone_id = static_cast<int>(parse_int(zid_t, false)); // DECIMAL

    // zone-type is a symbol (unquoted string)
    auto type_t = lexer_.next();
    std::string zone_type = type_t.text;

    // zone-name is a symbol
    auto name_t = lexer_.next();
    std::string zone_name = name_t.text;

    // domain-id (optional)
    int domain_id = 0;
    auto peek = lexer_.peek();
    if (peek.type == TokenType::INTEGER) {
        auto did_t = expect_int("zone domain-id");
        domain_id = static_cast<int>(parse_int(did_t, false));
    }

    expect(TokenType::RPAREN, "zone header close");

    // Skip remaining conditions body: ( (cond1 . val1) ... ) or just ()
    expect(TokenType::LPAREN, "zone conditions open");
    int depth = 1;
    while (depth > 0) {
        auto t = lexer_.next();
        if (t.type == TokenType::END_OF_FILE) break;
        if (t.type == TokenType::LPAREN) depth++;
        if (t.type == TokenType::RPAREN) depth--;
    }

    MeshData::ZoneInfo zi;
    zi.name = zone_name;
    zi.type = zone_type;
    zi.domain_id = domain_id;
    data_.zones[zone_id] = zi;

    in_grid_section_ = true;

    expect(TokenType::RPAREN, "zone section close");
}

// ============================================================
// Section 58: Cell Tree
// ============================================================
void Parser::parse_cell_tree() {
    expect(TokenType::LPAREN, "cell-tree header");

    auto cid0_t = expect_int("cell-tree cell-id0");
    int cid0 = static_cast<int>(parse_hex_int(cid0_t));
    auto cid1_t = expect_int("cell-tree cell-id1");
    int cid1 = static_cast<int>(parse_hex_int(cid1_t));
    auto pzid_t = expect_int("cell-tree parent-zone-id");
    int pzid = static_cast<int>(parse_hex_int(pzid_t));
    auto czid_t = expect_int("cell-tree child-zone-id");
    int czid = static_cast<int>(parse_hex_int(czid_t));

    expect(TokenType::RPAREN, "cell-tree header close");

    int count = cid1 - cid0 + 1;

    expect(TokenType::LPAREN, "cell-tree body open");
    std::vector<Token> body;
    read_section_body_tokens(body);

    size_t idx = 0;
    for (int i = 0; i < count && idx < body.size(); i++) {
        MeshData::CellTreeEntry entry;
        entry.parent_cell_id = cid0 + i;
        entry.parent_zone_id = pzid;
        entry.child_zone_id = czid;

        int nkids = static_cast<int>(parse_hex_int(body[idx]));
        idx++;
        for (int j = 0; j < nkids && idx < body.size(); j++) {
            entry.child_cell_ids.push_back(static_cast<int>(parse_hex_int(body[idx])));
            idx++;
        }
        data_.cell_trees.push_back(entry);
    }
    expect(TokenType::RPAREN, "cell-tree section close");
}

// ============================================================
// Section 59: Face Tree
// ============================================================
void Parser::parse_face_tree() {
    expect(TokenType::LPAREN, "face-tree header");

    auto fid0_t = expect_int("face-tree face-id0");
    int fid0 = static_cast<int>(parse_hex_int(fid0_t));
    auto fid1_t = expect_int("face-tree face-id1");
    int fid1 = static_cast<int>(parse_hex_int(fid1_t));
    auto pzid_t = expect_int("face-tree parent-zone-id");
    int pzid = static_cast<int>(parse_hex_int(pzid_t));
    auto czid_t = expect_int("face-tree child-zone-id");
    int czid = static_cast<int>(parse_hex_int(czid_t));

    expect(TokenType::RPAREN, "face-tree header close");

    int count = fid1 - fid0 + 1;

    expect(TokenType::LPAREN, "face-tree body open");
    std::vector<Token> body;
    read_section_body_tokens(body);

    size_t idx = 0;
    for (int i = 0; i < count && idx < body.size(); i++) {
        MeshData::FaceTreeEntry entry;
        entry.parent_face_id = fid0 + i;
        entry.parent_zone_id = pzid;
        entry.child_zone_id = czid;

        int nkids = static_cast<int>(parse_hex_int(body[idx]));
        idx++;
        for (int j = 0; j < nkids && idx < body.size(); j++) {
            entry.child_face_ids.push_back(static_cast<int>(parse_hex_int(body[idx])));
            idx++;
        }
        data_.face_trees.push_back(entry);
    }
    expect(TokenType::RPAREN, "face-tree section close");
}

// ============================================================
// Section 61: Interface Face Parents
// ============================================================
void Parser::parse_if_parents() {
    expect(TokenType::LPAREN, "if-parents header");

    auto fid0_t = expect_int("if-parents face-id0");
    int fid0 = static_cast<int>(parse_hex_int(fid0_t));
    auto fid1_t = expect_int("if-parents face-id1");
    int fid1 = static_cast<int>(parse_hex_int(fid1_t));

    expect(TokenType::RPAREN, "if-parents header close");

    int count = fid1 - fid0 + 1;

    expect(TokenType::LPAREN, "if-parents body open");
    std::vector<Token> body;
    read_section_body_tokens(body);

    // Each entry is a parent-id for each child face
    for (int i = 0; i < count && static_cast<size_t>(i) < body.size(); i++) {
        MeshData::IFParentEntry entry;
        entry.child_face_id = fid0 + i;
        entry.parent_face_ids.push_back(static_cast<int>(parse_hex_int(body[i])));
        data_.if_parents.push_back(entry);
    }
    expect(TokenType::RPAREN, "if-parents section close");
}

// ============================================================
// Main parse loop
// ============================================================
MeshData Parser::parse() {
    while (!lexer_.eof()) {
        auto t = lexer_.peek();
        if (t.type == TokenType::END_OF_FILE) break;

        if (t.type != TokenType::LPAREN) {
            // Expected section start
            auto err = lexer_.next();
            add_syntax_error("Unexpected token '" + err.text + "', expected '(' for section start");
            continue;
        }

        lexer_.next(); // consume '('

        // Read section index
        auto idx_t = lexer_.next();
        if (idx_t.type != TokenType::INTEGER) {
            add_syntax_error("Expected section index integer after '('");
            skip_section();
            continue;
        }

        int section_index = static_cast<int>(std::strtol(idx_t.text.c_str(), nullptr, 10));
        current_line_ = idx_t.line;

        in_grid_section_ = (section_index != 39 && section_index != 45);

        try {
            switch (section_index) {
                case 0:  parse_comment();              break;
                case 1:  parse_header();               break;
                case 2:  parse_dimensions();           break;
                case 10: parse_nodes();                break;
                case 11: skip_section();               break; // edges: skip
                case 12: parse_cells();                break;
                case 13: parse_faces();                break;
                case 18: parse_periodic_shadow_faces(); break;
                case 39:
                case 45: parse_zone(section_index);    break;
                case 40: skip_section();               break; // partitions: skip
                case 58: parse_cell_tree();            break;
                case 59: parse_face_tree();            break;
                case 61: parse_if_parents();           break;
                default:
                    add_syntax_warning("Unknown section index: " + std::to_string(section_index));
                    skip_section();
                    break;
            }
        } catch (const std::exception& e) {
            add_syntax_error(std::string("Parse error in section ") +
                           std::to_string(section_index) + ": " + e.what());
            // Try to recover by skipping to end of current section
            try { skip_section(); } catch (...) {}
        }
    }

    return std::move(data_);
}

} // namespace msh
