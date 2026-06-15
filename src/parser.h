#pragma once

#include "lexer.h"
#include "mesh_data.h"
#include <memory>
#include <string>
#include <functional>

namespace msh {

// ============================================================
// Parser: Token stream → MeshData
// ============================================================
class Parser {
public:
    explicit Parser(Lexer& lexer);

    MeshData parse();

    // Error callback: called when a parse error is encountered
    // Returning false aborts parsing; true continues with recovery
    using ErrorHandler = std::function<bool(const std::string& msg, size_t line)>;

    void set_error_handler(ErrorHandler handler) { error_handler_ = handler; }

    // Validation issues collected during parsing (syntax layer)
    const std::vector<ValidationIssue>& syntax_issues() const { return syntax_issues_; }

private:
    // Token helpers
    Token expect(TokenType type, const std::string& context);
    Token expect_int(const std::string& context);
    Token expect_float(const std::string& context);

    // Hex vs decimal parsing
    int64_t parse_int(const Token& t, bool hex);
    int64_t parse_hex_int(const Token& t);

    // Section dispatchers
    void skip_section();
    void parse_comment();
    void parse_header();
    void parse_dimensions();
    void parse_nodes();
    void parse_cells();
    void parse_faces();
    void parse_periodic_shadow_faces();
    void parse_zone(int section_index);
    void parse_face_tree();
    void parse_cell_tree();
    void parse_if_parents();

    // Section body helpers
    void read_section_body_tokens(std::vector<Token>& out);
    int read_paren_depth();

    Lexer& lexer_;
    MeshData data_;
    ErrorHandler error_handler_;

    // Track whether we're in a grid section (hex) or zone section (decimal)
    bool in_grid_section_ = true;

    // Line tracking for error reporting
    size_t current_line_ = 0;

    // Syntax issues collected
    std::vector<ValidationIssue> syntax_issues_;

    void add_syntax_error(const std::string& msg);
    void add_syntax_warning(const std::string& msg);
};

} // namespace msh
