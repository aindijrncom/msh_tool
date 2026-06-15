#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <sstream>

namespace msh {

// ANSI color codes
namespace color {
    constexpr const char* KEY    = "\033[34m";   // blue
    constexpr const char* STRING = "\033[32m";   // green
    constexpr const char* NUMBER = "\033[33m";   // yellow
    constexpr const char* BOOL   = "\033[35m";   // magenta
    constexpr const char* NULL_V = "\033[35m";   // magenta
    constexpr const char* RESET  = "\033[0m";
    constexpr const char* BRACKET = "\033[37m";  // white/gray
    constexpr const char* COMMA   = "\033[37m";
    constexpr const char* COLON   = "\033[37m";
}

// Recursively serialize JSON with ANSI colors
inline void colorize_value(const nlohmann::json& j, std::ostringstream& out, int indent, bool pretty);

inline void write_indent(std::ostringstream& out, int indent) {
    for (int i = 0; i < indent; i++) out << "  ";
}

inline void colorize_object(const nlohmann::json& j, std::ostringstream& out, int indent, bool pretty) {
    if (j.empty()) {
        out << color::BRACKET << "{}" << color::RESET;
        return;
    }
    out << color::BRACKET << "{" << color::RESET;
    if (pretty) out << '\n';
    bool first = true;
    for (auto& [key, val] : j.items()) {
        if (!first) {
            out << color::COMMA << "," << color::RESET;
            if (pretty) out << '\n';
        }
        first = false;
        if (pretty) write_indent(out, indent + 1);
        out << color::KEY << "\"" << key << "\"" << color::RESET
            << color::COLON << ": " << color::RESET;
        colorize_value(val, out, indent + 1, pretty);
    }
    if (pretty) {
        out << '\n';
        write_indent(out, indent);
    }
    out << color::BRACKET << "}" << color::RESET;
}

inline void colorize_array(const nlohmann::json& j, std::ostringstream& out, int indent, bool pretty) {
    if (j.empty()) {
        out << color::BRACKET << "[]" << color::RESET;
        return;
    }
    out << color::BRACKET << "[" << color::RESET;
    if (pretty) out << '\n';
    bool first = true;
    for (auto& val : j) {
        if (!first) {
            out << color::COMMA << "," << color::RESET;
            if (pretty) out << '\n';
        }
        first = false;
        if (pretty) write_indent(out, indent + 1);
        colorize_value(val, out, indent + 1, pretty);
    }
    if (pretty) {
        out << '\n';
        write_indent(out, indent);
    }
    out << color::BRACKET << "]" << color::RESET;
}

inline void colorize_value(const nlohmann::json& j, std::ostringstream& out, int indent, bool pretty) {
    if (j.is_object()) {
        colorize_object(j, out, indent, pretty);
    } else if (j.is_array()) {
        colorize_array(j, out, indent, pretty);
    } else if (j.is_string()) {
        out << color::STRING << "\"" << j.get<std::string>() << "\"" << color::RESET;
    } else if (j.is_number()) {
        out << color::NUMBER << j.dump() << color::RESET;
    } else if (j.is_boolean()) {
        out << color::BOOL << (j.get<bool>() ? "true" : "false") << color::RESET;
    } else if (j.is_null()) {
        out << color::NULL_V << "null" << color::RESET;
    }
}

inline std::string colorize_json(const nlohmann::json& j, bool pretty = true) {
    std::ostringstream out;
    colorize_value(j, out, 0, pretty);
    out << '\n';
    return out.str();
}

} // namespace msh
