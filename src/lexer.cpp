#include "lexer.h"
#include <cctype>
#include <cstdlib>
#include <stdexcept>

namespace msh {

Lexer::Lexer(std::istream& input) : in_(input) {
    advance();
}

void Lexer::advance() {
    current_char_ = in_.get();
    if (current_char_ == EOF || current_char_ == std::char_traits<char>::eof()) {
        eof_ = true;
        current_char_ = 0;
    }
    column_++;
}

void Lexer::skip_whitespace() {
    while (!eof_ && std::isspace(static_cast<unsigned char>(current_char_))) {
        if (current_char_ == '\n') {
            line_++;
            column_ = 0;
        }
        advance();
    }
}

Token Lexer::next() {
    if (has_peeked_) {
        has_peeked_ = false;
        return peeked_;
    }

    skip_whitespace();

    Token t;
    t.line = line_;
    t.column = column_;

    if (eof_) {
        t.type = TokenType::END_OF_FILE;
        return t;
    }

    char c = static_cast<char>(current_char_);

    if (c == '(') {
        t.type = TokenType::LPAREN;
        t.text = "(";
        advance();
        return t;
    }
    if (c == ')') {
        t.type = TokenType::RPAREN;
        t.text = ")";
        advance();
        return t;
    }
    if (c == '"') {
        return read_string();
    }

    // Everything else: word (INTEGER, FLOAT, or SYMBOL)
    return read_word();
}

Token Lexer::peek() {
    if (!has_peeked_) {
        peeked_ = next();
        has_peeked_ = true;
    }
    return peeked_;
}

Token Lexer::read_string() {
    Token t;
    t.type = TokenType::STRING;
    t.line = line_;
    t.column = column_;

    advance(); // skip opening '"'

    std::string result;
    while (!eof_ && current_char_ != '"') {
        if (current_char_ == '\\') {
            advance();
            if (!eof_) {
                result += static_cast<char>(current_char_);
                advance();
            }
        } else {
            result += static_cast<char>(current_char_);
            advance();
        }
    }

    if (!eof_) advance(); // skip closing '"'

    t.text = result;
    return t;
}

// ============================================================
// read_word: reads a contiguous run of non-whitespace,
// non-paren characters and classifies as INTEGER, FLOAT or SYMBOL.
// ============================================================
Token Lexer::read_word() {
    Token t;
    t.line = line_;
    t.column = column_;

    std::string raw;

    // Read until whitespace, paren, EOF
    while (!eof_ &&
           !std::isspace(static_cast<unsigned char>(current_char_)) &&
           current_char_ != '(' &&
           current_char_ != ')') {
        raw += static_cast<char>(current_char_);
        advance();
    }

    if (raw.empty()) {
        throw std::runtime_error("Empty token at line " + std::to_string(line_));
    }

    // Classify: try to parse as number first
    bool is_float = false;
    bool is_number = true;
    bool has_digit_or_hex = false;

    for (size_t i = 0; i < raw.size(); i++) {
        char ch = raw[i];
        if (i == 0 && (ch == '-' || ch == '+')) continue;
        if (ch == '.') { is_float = true; continue; }
        if ((ch == 'e' || ch == 'E') && is_float) continue;
        // After 'e'/'E' in a float, allow +/- for exponent
        if (i > 0 && (ch == '+' || ch == '-') && (raw[i-1] == 'e' || raw[i-1] == 'E') && is_float) continue;
        if (std::isdigit(static_cast<unsigned char>(ch))) { has_digit_or_hex = true; continue; }
        // Hex digits (a-f, A-F) — valid in pure hex integers only (not floats)
        if (!is_float && std::isxdigit(static_cast<unsigned char>(ch))) { has_digit_or_hex = true; continue; }
        // Not a valid number character → SYMBOL
        is_number = false;
        break;
    }

    if (is_number && has_digit_or_hex) {
        if (is_float) {
            t.type = TokenType::FLOAT;
            t.float_value = std::strtod(raw.c_str(), nullptr);
        } else {
            t.type = TokenType::INTEGER;
            t.int_value = std::strtoll(raw.c_str(), nullptr, 10);
        }
    } else {
        t.type = TokenType::SYMBOL;
    }

    t.text = raw;
    return t;
}

} // namespace msh
