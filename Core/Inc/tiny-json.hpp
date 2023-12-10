#pragma once

extern "C" {
    #include "tiny-json.h"
}
#include <array>
#include <optional>

#pragma GCC push_options
#pragma GCC optimize("no-inline")
#include "etl/string.h"
#include "etl/vector.h"
#pragma GCC pop_options

struct JSONObj {
    const json_t* ptr;
    JSONObj(const json_t* ptr);

    std::optional<int64_t> get_int(etl::string_view key);
    std::optional<int64_t> to_int();
    std::optional<etl::string_view> get_string(etl::string_view key);
    std::optional<etl::string_view> to_string();
    std::optional<bool> get_bool(etl::string_view key);
    std::optional<bool> to_bool();
    std::optional<JSONObj> get_obj(etl::string_view key);
    std::optional<JSONObj> to_1st_of_list(void);

    template <size_t N>
    std::optional<int> get_list(etl::string_view key, etl::vector<JSONObj, N>& list)
    {
        json_t const* field = json_getProperty(ptr, key.data());
        if (field == nullptr) {
            return std::nullopt;
        }
        if (json_getType(field) != JSON_ARRAY) {
            return std::nullopt;
        }

        int len = 0;
        json_t const* child;
        for (child = json_getChild(field); child != nullptr; child = json_getSibling(child)) {
            len++;
            list.emplace_back(JSONObj(child));
        }
        return len;
    }

    template <size_t N>
    std::optional<int> to_list(etl::vector<JSONObj, N>& list)
    {
        if (json_getType(ptr) != JSON_ARRAY) {
            return std::nullopt;
        }
        int len = 0;
        json_t const* child;
        for (child = json_getChild(ptr); child != nullptr; child = json_getSibling(child)) {
            len++;
            list.emplace_back(JSONObj(child));
        }
        return len;
    }
};

template <size_t N>
struct JSONParser {
    json_t pool[N];
    explicit JSONParser() {}
    json_t const* p_root;

    // json_createは失敗すると0を返す
    template <size_t K>
    bool parse(etl::string<K>& str)
    {
        str.initialize_free_space();
        p_root = json_create(str.data(), pool, N);
        if (p_root == 0) {
            return false;
        } else {
            return true;
        }
    }
    bool parse(char* str)
    {
        p_root = json_create(str, pool, N);
        if (p_root == 0) {
            return false;
        } else {
            return true;
        }
    }
    JSONObj root() const { return JSONObj(p_root); }
};
