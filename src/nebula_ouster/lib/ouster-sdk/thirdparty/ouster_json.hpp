// SPDX-License-Identifier: Apache-2.0
// Drop-in replacement for jsoncons, backed by nlohmann/json.
// Provides jsoncons::json, jsoncons::jsonpath::json_query, base64 encode/decode,
// and stub jsonschema — sufficient for the ouster-sdk usage surface.
#pragma once

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <istream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace jsoncons {

// ---------------------------------------------------------------------------
// Tag types and aliases
// ---------------------------------------------------------------------------

struct json_array_arg_t {};
constexpr json_array_arg_t json_array_arg{};

using string_view = std::string;
using ser_error = nlohmann::json::exception;

// ---------------------------------------------------------------------------
// Helper traits
// ---------------------------------------------------------------------------

template <typename T>
struct is_std_array : std::false_type {};
template <typename T, std::size_t N>
struct is_std_array<std::array<T, N>> : std::true_type {
    using value_type = T;
};

// Forward declaration for JsonKeyValue
class JsonKeyValue;

// ===========================================================================
// json class
// ===========================================================================

class json {
    std::shared_ptr<nlohmann::json> root_;
    nlohmann::json* ptr_;

    // Sub-reference constructor — shares root, points to a sub-node
    json(std::shared_ptr<nlohmann::json> root, nlohmann::json* ptr) noexcept
        : root_(std::move(root)), ptr_(ptr) {}

public:
    // -------------------------------------------------------------------------
    // Constructors
    // -------------------------------------------------------------------------

    // Default: null JSON value
    json() : root_(std::make_shared<nlohmann::json>()), ptr_(root_.get()) {}

    // Empty array
    explicit json(json_array_arg_t)
        : root_(std::make_shared<nlohmann::json>(nlohmann::json::array())),
          ptr_(root_.get()) {}

    // String value
    json(const std::string& s)
        : root_(std::make_shared<nlohmann::json>(s)), ptr_(root_.get()) {}

    json(std::string&& s)
        : root_(std::make_shared<nlohmann::json>(std::move(s))), ptr_(root_.get()) {}

    json(const char* s)
        : root_(std::make_shared<nlohmann::json>(s ? s : "")), ptr_(root_.get()) {}

    // std::vector<uint8_t> → JSON array of integers (NOT nlohmann binary type)
    json(const std::vector<uint8_t>& v) {
        auto arr = nlohmann::json::array();
        for (auto b : v) arr.push_back(static_cast<int>(b));
        root_ = std::make_shared<nlohmann::json>(std::move(arr));
        ptr_ = root_.get();
    }

    // Generic scalar (bool, int, double, …) — excludes types with dedicated ctors
    template <
        typename T,
        typename = std::enable_if_t<
            !std::is_same_v<std::decay_t<T>, json> &&
            !std::is_same_v<std::decay_t<T>, json_array_arg_t> &&
            !std::is_same_v<std::decay_t<T>, std::string> &&
            !std::is_same_v<std::decay_t<T>, std::vector<uint8_t>> &&
            !std::is_pointer_v<std::decay_t<T>>>>
    json(T&& val)
        : root_(std::make_shared<nlohmann::json>(std::forward<T>(val))),
          ptr_(root_.get()) {}

    // Copy constructor: always makes a deep independent copy
    json(const json& other)
        : root_(std::make_shared<nlohmann::json>(*other.ptr_)), ptr_(root_.get()) {}

    // Move constructor: transfer ownership; leave other in a valid empty state
    json(json&& other) noexcept
        : root_(std::move(other.root_)), ptr_(other.ptr_) {
        other.root_ = std::make_shared<nlohmann::json>();
        other.ptr_ = other.root_.get();
    }

    // -------------------------------------------------------------------------
    // Static factories
    // -------------------------------------------------------------------------

    static json parse(const std::string& s) {
        json j;
        *j.ptr_ = nlohmann::json::parse(s);
        return j;
    }

    static json parse(std::istream& is) {
        json j;
        is >> *j.ptr_;
        return j;
    }

    static json object() {
        json j;
        *j.ptr_ = nlohmann::json::object();
        return j;
    }

    // Internal factory: create a sub-reference into an existing shared tree
    static json from_ptr(std::shared_ptr<nlohmann::json> root,
                         nlohmann::json* ptr) noexcept {
        return json(std::move(root), ptr);
    }

    static json from_nlohmann(const nlohmann::json& n) {
        json j;
        *j.ptr_ = n;
        return j;
    }

    // -------------------------------------------------------------------------
    // Assignment
    // -------------------------------------------------------------------------

    json& operator=(const json& other) {
        if (this != &other) *ptr_ = *other.ptr_;
        return *this;
    }

    json& operator=(json&& other) {
        if (this != &other) *ptr_ = std::move(*other.ptr_);
        return *this;
    }

    template <typename T,
              typename = std::enable_if_t<!std::is_same_v<std::decay_t<T>, json>>>
    json& operator=(T&& val) {
        *ptr_ = std::forward<T>(val);
        return *this;
    }

    // -------------------------------------------------------------------------
    // Key / index access
    // -------------------------------------------------------------------------

    bool contains(const std::string& key) const noexcept {
        return ptr_->is_object() && ptr_->contains(key);
    }

    // Throws std::out_of_range if key is missing
    json at(const std::string& key) const {
        return json(root_, &ptr_->at(key));
    }

    // Mutable: auto-creates the key (null→object promotion)
    json operator[](const std::string& key) {
        if (ptr_->is_null()) *ptr_ = nlohmann::json::object();
        return json(root_, &(*ptr_)[key]);
    }

    // Const: returns null json if key is absent
    json operator[](const std::string& key) const {
        if (ptr_->is_object() && ptr_->contains(key))
            return json(root_, &(*ptr_)[key]);
        return json();
    }

    json operator[](std::size_t idx) {
        return json(root_, &(*ptr_)[idx]);
    }

    json operator[](std::size_t idx) const {
        return json(root_, &(*ptr_)[idx]);
    }

    // -------------------------------------------------------------------------
    // Type predicates
    // -------------------------------------------------------------------------

    bool is_null() const noexcept { return ptr_->is_null(); }
    bool is_array() const noexcept { return ptr_->is_array(); }
    bool is_object() const noexcept { return ptr_->is_object(); }
    bool is_number() const noexcept { return ptr_->is_number(); }
    bool is_string() const noexcept { return ptr_->is_string(); }
    bool is_boolean() const noexcept { return ptr_->is_boolean(); }
    bool empty() const { return ptr_->empty(); }
    std::size_t size() const { return ptr_->size(); }

    // Template type check — used by parse_and_validate_item<T>
    template <typename T>
    bool is() const noexcept {
        if constexpr (std::is_same_v<T, std::string>)
            return ptr_->is_string();
        else if constexpr (std::is_same_v<T, bool>)
            return ptr_->is_boolean();
        else if constexpr (std::is_same_v<T, double> || std::is_same_v<T, float>)
            return ptr_->is_number();
        else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>)
            // Accept both signed/unsigned integer JSON types; floats handled by
            // relaxed_number_verification in the caller
            return ptr_->is_number_integer() || ptr_->is_number_unsigned();
        else
            return !ptr_->is_null();
    }

    // -------------------------------------------------------------------------
    // Value extraction
    // -------------------------------------------------------------------------

    template <typename T>
    T as() const {
        if constexpr (std::is_same_v<T, std::string> ||
                      std::is_same_v<T, string_view>) {
            if (ptr_->is_string()) return ptr_->get<std::string>();
            return ptr_->dump();
        } else if constexpr (std::is_same_v<T, bool>) {
            return ptr_->get<bool>();
        } else if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {
            // Cross-type numeric coercion
            if (ptr_->is_number_integer())
                return static_cast<T>(ptr_->get<std::int64_t>());
            if (ptr_->is_number_unsigned())
                return static_cast<T>(ptr_->get<std::uint64_t>());
            if (ptr_->is_number_float())
                return static_cast<T>(ptr_->get<double>());
            if (ptr_->is_string()) {
                try {
                    return static_cast<T>(std::stod(ptr_->get<std::string>()));
                } catch (...) {}
            }
            throw std::runtime_error("json::as<T>(): cannot convert to numeric");
        } else if constexpr (std::is_same_v<T, std::vector<std::uint8_t>>) {
            std::vector<std::uint8_t> result;
            if (ptr_->is_array()) {
                for (const auto& elem : *ptr_) {
                    if (elem.is_number_integer())
                        result.push_back(static_cast<std::uint8_t>(elem.get<std::int64_t>()));
                    else if (elem.is_number_unsigned())
                        result.push_back(static_cast<std::uint8_t>(elem.get<std::uint64_t>()));
                    else if (elem.is_number_float())
                        result.push_back(static_cast<std::uint8_t>(elem.get<double>()));
                }
            } else if (ptr_->is_binary()) {
                auto& bin = ptr_->get_binary();
                result.assign(bin.begin(), bin.end());
            }
            return result;
        } else if constexpr (is_std_array<T>::value) {
            using VT = typename is_std_array<T>::value_type;
            T result{};
            if (ptr_->is_array()) {
                std::size_t i = 0;
                for (const auto& elem : *ptr_) {
                    if (i >= result.size()) break;
                    result[i++] = json::from_nlohmann(elem).template as<VT>();
                }
            }
            return result;
        } else {
            // Let nlohmann handle everything else (std::vector<T>, etc.)
            return ptr_->template get<T>();
        }
    }

    double as_double() const { return as<double>(); }

    template <typename T>
    T get_value_or(const std::string& key, const T& def) const noexcept {
        if (!ptr_->is_object() || !ptr_->contains(key)) return def;
        try {
            return json(root_, &ptr_->at(key)).template as<T>();
        } catch (...) {
            return def;
        }
    }

    // -------------------------------------------------------------------------
    // Mutation
    // -------------------------------------------------------------------------

    void emplace_back(const json& val) { ptr_->push_back(*val.ptr_); }
    void emplace_back(json&& val) { ptr_->push_back(std::move(*val.ptr_)); }

    template <typename T,
              typename = std::enable_if_t<!std::is_same_v<std::decay_t<T>, json>>>
    void emplace_back(T&& val) {
        ptr_->push_back(std::forward<T>(val));
    }

    // -------------------------------------------------------------------------
    // Serialization
    // -------------------------------------------------------------------------

    void dump(std::string& out) const { out = ptr_->dump(); }
    void dump_pretty(std::string& out) const { out = ptr_->dump(4); }
    std::string to_string() const { return ptr_->dump(); }

    std::string type() const {
        if (ptr_->is_null()) return "null";
        if (ptr_->is_boolean()) return "boolean";
        if (ptr_->is_number()) return "number";
        if (ptr_->is_string()) return "string";
        if (ptr_->is_array()) return "array";
        if (ptr_->is_object()) return "object";
        return "unknown";
    }

    // -------------------------------------------------------------------------
    // Comparison
    // -------------------------------------------------------------------------

    bool operator==(const json& other) const { return *ptr_ == *other.ptr_; }
    bool operator!=(const json& other) const { return *ptr_ != *other.ptr_; }

    template <typename T,
              typename = std::enable_if_t<!std::is_same_v<std::decay_t<T>, json>>>
    bool operator==(const T& val) const noexcept {
        try { return *ptr_ == val; } catch (...) { return false; }
    }

    template <typename T,
              typename = std::enable_if_t<!std::is_same_v<std::decay_t<T>, json>>>
    bool operator!=(const T& val) const noexcept {
        return !(*this == val);
    }

    friend std::ostream& operator<<(std::ostream& os, const json& j) {
        return os << j.ptr_->dump();
    }

    // -------------------------------------------------------------------------
    // Iteration (object_range defined after JsonKeyValue below)
    // -------------------------------------------------------------------------

    auto object_range() const;  // -> std::vector<JsonKeyValue>

    std::vector<json> array_range() const {
        std::vector<json> result;
        if (ptr_->is_array()) {
            for (auto& elem : *ptr_)
                result.emplace_back(json(root_, &elem));
        }
        return result;
    }

    // -------------------------------------------------------------------------
    // Internal accessors (used by jsonpath implementation below)
    // -------------------------------------------------------------------------

    const nlohmann::json* nlohmann_ptr() const noexcept { return ptr_; }
    nlohmann::json* nlohmann_ptr() noexcept { return ptr_; }
    const std::shared_ptr<nlohmann::json>& nlohmann_root() const noexcept {
        return root_;
    }
};

// ===========================================================================
// JsonKeyValue — one entry from object_range()
// ===========================================================================

class JsonKeyValue {
public:
    JsonKeyValue(std::string key, json val)
        : key_(std::move(key)), value_(std::move(val)) {}

    const std::string& key() const noexcept { return key_; }
    json& value() noexcept { return value_; }
    const json& value() const noexcept { return value_; }

private:
    std::string key_;
    json value_;
};

// Define object_range() after JsonKeyValue is complete
inline auto json::object_range() const {
    std::vector<JsonKeyValue> result;
    if (ptr_->is_object()) {
        for (auto it = ptr_->begin(); it != ptr_->end(); ++it)
            result.emplace_back(it.key(), json(root_, &it.value()));
    }
    return result;
}

// ===========================================================================
// Base64 encode / decode
// ===========================================================================

namespace detail {
static constexpr char b64_enc[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static constexpr std::int8_t b64_dec[256] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,  // 0-15
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,  // 16-31
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, -1, -1, 63,  // 32-47
    52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1,  0, -1, -1,  // 48-63
    -1,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,  // 64-79
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1,  // 80-95
    -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,  // 96-111
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, -1, -1, -1, -1, -1,  // 112-127
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};
}  // namespace detail

template <typename It>
void encode_base64(It begin, It end, std::string& out) {
    out.clear();
    int val = 0, valb = -6;
    for (auto it = begin; it != end; ++it) {
        val = (val << 8) + static_cast<std::uint8_t>(*it);
        valb += 8;
        while (valb >= 0) {
            out.push_back(detail::b64_enc[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }
    if (valb > -6)
        out.push_back(detail::b64_enc[((val << 8) >> (valb + 8)) & 0x3F]);
    while (out.size() % 4) out.push_back('=');
}

template <typename It>
void decode_base64(It begin, It end, std::vector<std::uint8_t>& out) {
    out.clear();
    int val = 0, valb = -8;
    for (auto it = begin; it != end; ++it) {
        std::int8_t d =
            detail::b64_dec[static_cast<std::uint8_t>(static_cast<char>(*it))];
        if (d < 0) continue;  // skip padding and invalid chars
        val = (val << 6) + d;
        valb += 6;
        if (valb >= 0) {
            out.push_back(static_cast<std::uint8_t>((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
}

// ===========================================================================
// JSONPath (subset: $.a.b.c, $.a.b.*, $.a[N], $.'quoted'.b)
// ===========================================================================

namespace jsonpath {
namespace impl {

struct Segment {
    enum class Type { KEY, INDEX, WILDCARD };
    Type type;
    std::string key;
    std::size_t index = 0;
    std::string path_token;  // the string that was parsed (for round-trip)
};

inline std::vector<Segment> parse_path(const std::string& path) {
    std::vector<Segment> segs;
    std::size_t pos = 0;
    if (path.empty() || path[0] != '$') return segs;
    pos = 1;

    while (pos < path.size()) {
        if (path[pos] == '.') {
            ++pos;
            if (pos >= path.size()) break;
            if (path[pos] == '*') {
                segs.push_back({Segment::Type::WILDCARD, {}, 0, ".*"});
                ++pos;
            } else if (path[pos] == '\'') {
                // Quoted key: $.'some-key'
                ++pos;
                auto end = path.find('\'', pos);
                if (end == std::string::npos) end = path.size();
                std::string k = path.substr(pos, end - pos);
                segs.push_back({Segment::Type::KEY, k, 0, ".'" + k + "'"});
                pos = (end < path.size()) ? end + 1 : end;
            } else {
                auto end = pos;
                while (end < path.size() && path[end] != '.' && path[end] != '[')
                    ++end;
                std::string k = path.substr(pos, end - pos);
                segs.push_back({Segment::Type::KEY, k, 0, "." + k});
                pos = end;
            }
        } else if (path[pos] == '[') {
            ++pos;
            auto end = path.find(']', pos);
            if (end == std::string::npos) break;
            std::string tok = path.substr(pos, end - pos);
            pos = end + 1;
            if (tok == "*") {
                segs.push_back({Segment::Type::WILDCARD, {}, 0, "[*]"});
            } else {
                std::size_t idx = static_cast<std::size_t>(std::stoul(tok));
                segs.push_back({Segment::Type::INDEX, {}, idx, "[" + tok + "]"});
            }
        } else {
            break;
        }
    }
    return segs;
}

using MatchList =
    std::vector<std::pair<std::string, nlohmann::json*>>;

inline void evaluate(nlohmann::json* node, const std::vector<Segment>& segs,
                     std::size_t si, const std::string& cur, MatchList& out) {
    if (!node) return;
    if (si == segs.size()) {
        out.push_back({cur, node});
        return;
    }
    const auto& seg = segs[si];
    if (seg.type == Segment::Type::KEY) {
        if (node->is_object() && node->contains(seg.key))
            evaluate(&(*node)[seg.key], segs, si + 1, cur + seg.path_token, out);
    } else if (seg.type == Segment::Type::INDEX) {
        if (node->is_array() && seg.index < node->size())
            evaluate(&(*node)[seg.index], segs, si + 1, cur + seg.path_token, out);
    } else {  // WILDCARD
        if (node->is_array()) {
            for (std::size_t i = 0; i < node->size(); ++i)
                evaluate(&(*node)[i], segs, si + 1,
                         cur + "[" + std::to_string(i) + "]", out);
        } else if (node->is_object()) {
            for (auto it = node->begin(); it != node->end(); ++it)
                evaluate(&it.value(), segs, si + 1, cur + "." + it.key(), out);
        }
    }
}

}  // namespace impl

// Non-callback form: returns array of matches
inline json json_query(const json& root, const std::string& path) {
    auto segs = impl::parse_path(path);
    impl::MatchList matches;
    impl::evaluate(const_cast<nlohmann::json*>(root.nlohmann_ptr()), segs, 0,
                   "$", matches);

    json result(json_array_arg);
    for (auto& [p, node] : matches)
        result.emplace_back(json::from_nlohmann(*node));
    return result;
}

// Callback form: invokes callback(path_string, json_val) for each match
template <typename F>
void json_query(const json& root, const std::string& path, F callback) {
    auto segs = impl::parse_path(path);
    impl::MatchList matches;
    impl::evaluate(const_cast<nlohmann::json*>(root.nlohmann_ptr()), segs, 0,
                   "$", matches);

    for (auto& [p, node] : matches) {
        json val = json::from_ptr(root.nlohmann_root(),
                                  const_cast<nlohmann::json*>(node));
        callback(p, static_cast<const json&>(val));
    }
}

}  // namespace jsonpath

// ===========================================================================
// JSON Schema — stub (zone monitoring is not used in this integration)
// ===========================================================================

namespace jsonschema {

struct json_schema {};

inline json_schema make_schema(const json& /*schema_json*/) { return {}; }

template <typename Json>
struct json_validator {
    explicit json_validator(const json_schema& /*s*/) {}
    // No-op: ouster zone monitoring schema validation is not required here
    void validate(const Json& /*instance*/) const noexcept {}
};

}  // namespace jsonschema

}  // namespace jsoncons
