#include "tiny-json.hpp"

JSONObj::JSONObj(const json_t* ptr) : ptr(ptr) {}

std::optional<int64_t> JSONObj::get_int(etl::string_view key)
{
    json_t const* field = json_getProperty(ptr, key.data());
    if (field == nullptr) {
        return std::nullopt;
    }
    if (json_getType(field) != JSON_INTEGER) {
        return std::nullopt;
    }

    return json_getInteger(field);
}

std::optional<int64_t> JSONObj::to_int()
{
    if (json_getType(ptr) != JSON_INTEGER) {
        return std::nullopt;
    }
    return json_getInteger(ptr);
}

std::optional<etl::string_view> JSONObj::get_string(etl::string_view key)
{
    json_t const* field = json_getProperty(ptr, key.data());
    if (field == nullptr) {
        return std::nullopt;
    }
    if (json_getType(field) != JSON_TEXT) {
        return std::nullopt;
    }

    return json_getValue(field);
}

std::optional<etl::string_view> JSONObj::to_string()
{
    if (json_getType(ptr) != JSON_TEXT) {
        return std::nullopt;
    }
    return json_getValue(ptr);
}

std::optional<bool> JSONObj::get_bool(etl::string_view key)
{
    json_t const* field = json_getProperty(ptr, key.data());
    if (field == nullptr) {
        return std::nullopt;
    }
    if (json_getType(field) != JSON_BOOLEAN) {
        return std::nullopt;
    }

    return json_getBoolean(field);
}

std::optional<bool> JSONObj::to_bool()
{
    if (json_getType(ptr) != JSON_BOOLEAN) {
        return std::nullopt;
    }
    return json_getBoolean(ptr);
}

std::optional<JSONObj> JSONObj::get_obj(etl::string_view key)
{
    json_t const* field = json_getProperty(ptr, key.data());
    if (field == nullptr) {
        return std::nullopt;
    }
    if (json_getType(field) != JSON_OBJ) {
        return std::nullopt;
    }

    return JSONObj(field);
}

std::optional<JSONObj> JSONObj::to_1st_of_list(void)
{
    if (json_getType(ptr) != JSON_ARRAY) {
        return std::nullopt;
    }
    json_t const* child = json_getChild(ptr);
    if (nullptr == child) {  // Null pointer if the json object has not properties
        return std::nullopt;
    }
    return JSONObj(child);
}

std::optional<int> JSONObj::findArray(etl::string_view key)
{
    // ルートオブジェクトから name フィールドを取得
    json_t const* field = json_getProperty(ptr, key.data());
    if (field == nullptr) {
        return std::nullopt;
    }
    if (JSON_ARRAY != json_getType(field)) {
        return std::nullopt;
    }

    json_t const* arrayItem;
    arrayItem = json_getChild(field);
    if (0 == arrayItem) {
        return std::nullopt;
    }
    int counter = 1;
    for (; arrayItem != 0; arrayItem = json_getSibling(arrayItem), counter++) {
        //           if ( JSON_OBJ == json_getType( phone ) ) {
        //               char const* phoneNumber = json_getPropertyValue( phone, "number" );
        //               if ( phoneNumber ) printf( "Number: %s.\n", phoneNumber );
        //           }
    }
    return counter;
}
