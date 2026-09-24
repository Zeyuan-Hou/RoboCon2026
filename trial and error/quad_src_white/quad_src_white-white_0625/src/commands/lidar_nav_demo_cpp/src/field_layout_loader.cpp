#include "lidar_nav_demo_cpp/field_layout_loader.hpp"

#include <cctype>
#include <fstream>
#include <sstream>

namespace lidar_nav {
namespace {

std::string strip_inline_yaml_comment(const std::string& line) {
    bool in_single = false;
    bool in_double = false;
    std::string out;
    out.reserve(line.size());
    for (char ch : line) {
        if (ch == '"' && !in_single) {
            in_double = !in_double;
        } else if (ch == '\'' && !in_double) {
            in_single = !in_single;
        } else if (ch == '#' && !in_single && !in_double) {
            break;
        }
        out.push_back(ch);
    }
    return out;
}

bool parse_numbers_from_text(const std::string& text, std::vector<double>& out) {
    out.clear();
    const size_t n = text.size();
    size_t i = 0;
    while (i < n) {
        while (i < n && !std::isdigit(static_cast<unsigned char>(text[i])) &&
               text[i] != '-' && text[i] != '+' && text[i] != '.') {
            ++i;
        }
        if (i >= n) {
            break;
        }

        const size_t start = i;
        if (text[i] == '-' || text[i] == '+') {
            ++i;
        }

        bool saw_digit = false;
        while (i < n) {
            const char c = text[i];
            if (std::isdigit(static_cast<unsigned char>(c))) {
                saw_digit = true;
                ++i;
                continue;
            }
            if (c == '.' && (i + 1 >= n || text[i + 1] != '.')) {
                saw_digit = true;
                ++i;
                continue;
            }
            if ((c == 'e' || c == 'E') && saw_digit) {
                ++i;
                if (i < n && (text[i] == '+' || text[i] == '-')) {
                    ++i;
                }
                while (i < n && std::isdigit(static_cast<unsigned char>(text[i]))) {
                    ++i;
                }
                break;
            }
            break;
        }

        if (!saw_digit) {
            ++i;
            continue;
        }

        try {
            out.push_back(std::stod(text.substr(start, i - start)));
        } catch (const std::exception&) {
            return false;
        }
    }
    return !out.empty();
}

bool find_matching_bracket(const std::string& content, size_t open_pos, size_t& close_pos) {
    if (open_pos >= content.size() || content[open_pos] != '[') {
        return false;
    }
    int depth = 0;
    for (size_t i = open_pos; i < content.size(); ++i) {
        if (content[i] == '[') {
            ++depth;
        } else if (content[i] == ']') {
            --depth;
            if (depth == 0) {
                close_pos = i;
                return true;
            }
        }
    }
    return false;
}

bool extract_array_after_key(
    const std::string& content, const std::string& key, std::vector<double>& out)
{
    const std::string needle = key + ":";
    const auto pos = content.find(needle);
    if (pos == std::string::npos) {
        return false;
    }

    const auto open_bracket = content.find('[', pos);
    if (open_bracket == std::string::npos) {
        return false;
    }

    size_t close_bracket = std::string::npos;
    if (!find_matching_bracket(content, open_bracket, close_bracket)) {
        return false;
    }

    std::string body;
    body.reserve(close_bracket - open_bracket);
    size_t cursor = open_bracket + 1;
    while (cursor < close_bracket) {
        size_t line_end = content.find('\n', cursor);
        if (line_end == std::string::npos || line_end > close_bracket) {
            line_end = close_bracket;
        }
        body += strip_inline_yaml_comment(content.substr(cursor, line_end - cursor));
        body += ' ';
        cursor = (line_end < close_bracket) ? line_end + 1 : close_bracket;
    }

    return parse_numbers_from_text(body, out);
}

}  // namespace

bool load_generated_manip_points(
    const std::string& path, GeneratedManipPoints& out, std::string& error)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        error = "cannot open file: " + path;
        return false;
    }

    std::ostringstream buffer;
    buffer << in.rdbuf();
    const std::string content = buffer.str();

    std::vector<double> pickup;
    std::vector<double> dropoff;
    if (!extract_array_after_key(content, "pickup_points", pickup)) {
        error = "pickup_points not found or invalid in " + path;
        return false;
    }
    if (!extract_array_after_key(content, "dropoff_points", dropoff)) {
        error = "dropoff_points not found or invalid in " + path;
        return false;
    }
    if (pickup.size() < 32) {
        error = "pickup_points need 32 values (8x4), got " + std::to_string(pickup.size());
        return false;
    }
    if (dropoff.size() < 24) {
        error = "dropoff_points need 24 values (8x3), got " + std::to_string(dropoff.size());
        return false;
    }

    out.pickup_flat.assign(pickup.begin(), pickup.begin() + 32);
    out.dropoff_flat.assign(dropoff.begin(), dropoff.begin() + 24);
    return true;
}

}  // namespace lidar_nav
