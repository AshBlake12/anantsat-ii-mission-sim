// INI-style configuration: "[section]" headers and "key = value" lines.
// Values are looked up as "section.key". Unknown keys are reported.
#pragma once
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cs {

inline std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

inline std::vector<double> parse_list(const std::string& s) {
    std::vector<double> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (!item.empty()) out.push_back(std::stod(item));
    }
    return out;
}

class Ini {
public:
    std::map<std::string, std::string> kv;
    // Ordered entries per section, for list-like sections such as [targets].
    std::map<std::string, std::vector<std::pair<std::string, std::string>>> sections;
    mutable std::set<std::string> used;
    std::string raw;  // original text

    bool load(const std::string& path) {
        std::ifstream f(path);
        if (!f) return false;
        std::stringstream ss;
        ss << f.rdbuf();
        parse(ss.str(), path);
        return true;
    }

    void parse(const std::string& text, const std::string& path = "config") {
        raw = text;
        std::istringstream f(text);
        std::string line, sec;
        int ln = 0;
        while (std::getline(f, line)) {
            ++ln;
            size_t hash = line.find('#');
            if (hash != std::string::npos) line = line.substr(0, hash);
            line = trim(line);
            if (line.empty()) continue;
            if (line.front() == '[') {
                sec = trim(line.substr(1, line.find(']') - 1));
                continue;
            }
            size_t eq = line.find('=');
            if (eq == std::string::npos)
                throw std::runtime_error(path + ":" + std::to_string(ln) + ": expected key = value");
            std::string k = trim(line.substr(0, eq)), v = trim(line.substr(eq + 1));
            kv[sec + "." + k] = v;
            sections[sec].push_back({k, v});
        }
    }

    bool has(const std::string& k) const { return kv.count(k) > 0; }
    std::string str(const std::string& k, const std::string& def) const {
        used.insert(k);
        auto it = kv.find(k);
        return it == kv.end() ? def : it->second;
    }
    double num(const std::string& k, double def) const {
        used.insert(k);
        auto it = kv.find(k);
        if (it == kv.end()) return def;
        try {
            return std::stod(it->second);
        } catch (...) {
            throw std::runtime_error("config: '" + k + "' is not a number: " + it->second);
        }
    }
    bool flag(const std::string& k, bool def) const {
        std::string v = str(k, def ? "true" : "false");
        return v == "true" || v == "1" || v == "yes" || v == "on";
    }
    std::vector<double> list(const std::string& k, const std::vector<double>& def) const {
        used.insert(k);
        auto it = kv.find(k);
        return it == kv.end() ? def : parse_list(it->second);
    }
    std::string warn_unused(const std::set<std::string>& list_sections) const {
        std::string out;
        for (auto& [k, v] : kv) {
            std::string sec = k.substr(0, k.find('.'));
            if (list_sections.count(sec)) continue;
            if (!used.count(k)) out += "warning: unknown config key '" + k + "'\n";
        }
        return out;
    }
};

}  // namespace cs
