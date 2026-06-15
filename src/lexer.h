#pragma once

#include <istream>
#include <string>
#include <variant>
#include <cstdint>

namespace msh {

// ============================================================
// Token types
// ============================================================
enum class TokenType {
    LPAREN,      // (
    RPAREN,      // )
    INTEGER,     // decimal or hex (raw string preserved)
    FLOAT,       // scientific notation float
    STRING,      // "quoted string"
    SYMBOL,      // unquoted symbol (e.g. zone names, types)
    END_OF_FILE
};

struct Token {
    TokenType type;
    std::string text;       // raw text for INTEGER/FLOAT/STRING
    int64_t int_value = 0;  // parsed integer (caller decides hex vs dec)
    double float_value = 0; // parsed float
    size_t line = 0;
    size_t column = 0;
};

// ============================================================
// Lexer: S-expression tokenizer
// ============================================================
class Lexer {
public:
    explicit Lexer(std::istream& input);

    Token next();
    Token peek();

    size_t line() const   { return line_; }
    size_t column() const { return column_; }
    bool eof() const      { return eof_; }

private:
    void advance();
    void skip_whitespace();
    Token read_number();
    Token read_word();    // INTEGER, FLOAT, or SYMBOL
    Token read_string();

    std::istream& in_;
    size_t line_ = 1;
    size_t column_ = 1;
    int current_char_ = 0;
    bool eof_ = false;
    bool has_peeked_ = false;
    Token peeked_;
};

} // namespace msh
