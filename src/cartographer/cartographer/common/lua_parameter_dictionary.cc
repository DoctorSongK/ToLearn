/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// When a LuaParameterDictionary is constructed, a new Lua state (i.e. an
// independent Lua interpreter) is fired up to evaluate the Lua code. The code
// is expected to return a Lua table that contains key/value pairs that are the
// key/value pairs of our parameter dictionary.
//
// We keep the Lua interpreter around and the table on the stack and reference
// it in the Get* methods instead of moving its contents from Lua into a C++ map
// since we can only know in the Get* methods what data type to expect in the
// table.
//
// Some of the methods below documentation the current stack with the following
// notation: S: <bottom> ... <top>

#include "cartographer/common/lua_parameter_dictionary.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

namespace cartographer {
namespace common {

namespace {

// Replace the string at the top of the stack through a quoted version that Lua
// can read back.
void QuoteStringOnStack(lua_State* L) {
  CHECK(lua_isstring(L, -1)) << "Top of stack is not a string value.";
  int current_index = lua_gettop(L);

  // S: ... string
  lua_pushglobaltable(L);         // S: ... string globals
  lua_getfield(L, -1, "string");  // S: ... string globals <string module>
  lua_getfield(L, -1,
               "format");   // S: ... string globals <string module> format
  lua_pushstring(L, "%q");  // S: ... string globals <string module> format "%q"
  lua_pushvalue(L, current_index);  // S: ... string globals <string module>
                                    // format "%q" string

  lua_call(L, 2, 1);  // S: ... string globals <string module> quoted
  lua_replace(L, current_index);  // S: ... quoted globals <string module>

  lua_pop(L, 2);  // S: ... quoted
}

// Sets the given 'dictionary' as the value of the "this" key in Lua's registry
// table.
// 将给定的“dictionary”设置为Lua注册表表中“this”键的值
void SetDictionaryInRegistry(lua_State* L, LuaParameterDictionary* dictionary) {
  //lua: const char *lua_pushstring (lua_State *L, const char *s);
  // 将指针s指向的零结尾的字符串压栈。lua对这个字符串做一个内部副本（或是复用一个副本），因此s的内存在函数返回后，可以释放掉或立刻用于其他用途
  // 返回内部副本的指针。  如s为null,将nil压栈并返回NULL
  lua_pushstring(L, "this");
  // lua: void lua_pushlightuserdata (lua_State *L, void *p);
  // 把一个轻量用户数据压栈。用户数据是保留在lua中的C值。轻量用户数据表示一个指针void*。它是一个像数字一样的值；你不需要专门创建它，也没有独立的元素，而且也不会被收集（因为从来
  // 不需要创建）。只要表示的c地址相同，两个轻量用户数据就相等。
  lua_pushlightuserdata(L, dictionary);
  // lua: void lua_settable (lua_State *L, int index);
  // 把表在lua堆栈中的值弹出来，index 是table 在堆栈中的位置，假如 table 在 -3, 则key 应该是 -2，value 是 -1
  // 相当于 table[key] = value,这个函数会将键和值都弹出栈。
  // NOTE: 通过阅读后面的代码明白了为什么这边设置table时是在一个很远的位置，是防止后期往栈中push数据的时候产生冲突
  lua_settable(L, LUA_REGISTRYINDEX);
}

// Gets the 'dictionary' from the "this" key in Lua's registry table.
/**
 * @brief Get the Dictionary From Registry object 从刚才设计的登记表导入的字典中获取对应字典
 * 
 * @param[in] L 
 * @return LuaParameterDictionary* 
 */
LuaParameterDictionary* GetDictionaryFromRegistry(lua_State* L) {
  lua_pushstring(L, "this");
  lua_gettable(L, LUA_REGISTRYINDEX);
  void* return_value = lua_isnil(L, -1) ? nullptr : lua_touserdata(L, -1);

  // lua: 从栈中弹出n个元素
  lua_pop(L, 1);
  CHECK(return_value != nullptr);
  return reinterpret_cast<LuaParameterDictionary*>(return_value);
}

// CHECK()s if a Lua method returned an error.
///@brief 用于检查lua中函数是否返回一个错误
void CheckForLuaErrors(lua_State* L, int status) {
  CHECK_EQ(status, 0) << lua_tostring(L, -1);
}

// Returns 'a' if 'condition' is true, else 'b'.
int LuaChoose(lua_State* L) {

  //lua: int lua_gettop (lua_State *L);
  // 返回栈顶元素的索引。 因为索引是从 1 开始编号的， 所以这个结果等于栈上的元素个数； 特别指出，0 表示栈为空。 
  CHECK_EQ(lua_gettop(L), 3) << "choose() takes (condition, a, b).";

  // lua: int lua_isboolean (lua_State *L, int index);
  // 当给定索引的值是一个布尔量时，返回1，否则返回0
  CHECK(lua_isboolean(L, 1)) << "condition is not a boolean value.";

  // lua: int lua_toboolean (lua_State *L, int index);
  // 把给定索引处的 Lua 值转换为一个 C 中的布尔量（ 0 或是 1 ）。
  const bool condition = lua_toboolean(L, 1);
  if (condition) {
    lua_pushvalue(L, 2);
  } else {
    lua_pushvalue(L, 3);
  }
  return 1;
}

// Pushes a value to the Lua stack.
void PushValue(lua_State* L, const int key) { lua_pushinteger(L, key); }
void PushValue(lua_State* L, const std::string& key) {
  lua_pushstring(L, key.c_str());
}

// Reads the value with the given 'key' from the Lua dictionary and pushes it to
// the top of the stack.
template <typename T>
void GetValueFromLuaTable(lua_State* L, const T& key) {
  PushValue(L, key);
  lua_rawget(L, -2);
}

// CHECK() that the topmost parameter on the Lua stack is a table.
void CheckTableIsAtTopOfStack(lua_State* L) {
  CHECK(lua_istable(L, -1)) << "Topmost item on Lua stack is not a table!";
}

// Returns true if 'key' is in the table at the top of the Lua stack.
template <typename T>
bool HasKeyOfType(lua_State* L, const T& key) {
  CheckTableIsAtTopOfStack(L);
  PushValue(L, key);
  lua_rawget(L, -2);
  const bool key_not_found = lua_isnil(L, -1);
  lua_pop(L, 1);  // Pop the item again.
  return !key_not_found;
}

// Iterates over the integer keys of the table at the top of the stack of 'L•
// and pushes the values one by one. 'pop_value' is expected to pop a value and
// put them into a C++ container.
void GetArrayValues(lua_State* L, const std::function<void()>& pop_value) {
  int idx = 1;
  while (true) {
    GetValueFromLuaTable(L, idx);
    if (lua_isnil(L, -1)) {
      lua_pop(L, 1);
      break;
    }
    pop_value();
    ++idx;
  }
}

}  // namespace

// 更多Lua的学习可以参考 https://www.cnblogs.com/chevin/p/5884657.html

std::unique_ptr<LuaParameterDictionary>
LuaParameterDictionary::NonReferenceCounted(
    const std::string& code, std::unique_ptr<FileResolver> file_resolver) {
  return std::unique_ptr<LuaParameterDictionary>(new LuaParameterDictionary(
      code, ReferenceCount::NO, std::move(file_resolver)));
}

/**
 * @brief Construct a new Lua Parameter Dictionary:: Lua Parameter Dictionary object
 * 
 * @param[in] code 配置文件内容
 * @param[in] file_resolver FileResolver类
 */
LuaParameterDictionary::LuaParameterDictionary(
    const std::string& code, std::unique_ptr<FileResolver> file_resolver)
    : LuaParameterDictionary(code, ReferenceCount::YES,
                             std::move(file_resolver)) {}

/**
 * @brief Construct a new Lua Parameter Dictionary:: Lua Parameter Dictionary object
 *        根据给定的字符串, 生成一个lua字典
 * 
 * @param[in] code 配置文件内容
 * @param[in] reference_count 
 * @param[in] file_resolver FileResolver类
 */
// lua: int lua_getglobal (lua_State *L, const char *name);
// 把全局变量 name 里的值压栈，返回该值的类型。
// lua: void lua_setglobal (lua_State *L, const char *name);
// 从堆栈上弹出一个值，并将其设为全局变量 name 的新值。
// lua: int lua_getfield (lua_State *L, int index, const char *k);
// 把 t[k] 的值压栈， 这里的 t 是索引指向的值。 在 Lua 中，这个函数可能触发对应 "index" 事件对应的元方法 （参见 §2.4 ）。
// 函数将返回压入值的类型。

// Lua使用一个虚拟栈来和C互传值。栈上的每个元素都是一个lua值（nil, 数字， 字符串，应该也是包含函数（c <--> lua））
//lua: lua_State *lua_newstate (lua_Alloc f, void *ud); 创建一个运行在新的独立的状态机中的线程
// 如果无法创建线程或状态机（由于内存有限）则返回NULL。参数f是一个分配器函数；lua通过这个函数做状态机内所有的内存操作；
// 第二个参数ud，这个指针将在每次调用分配器是被转入。
LuaParameterDictionary::LuaParameterDictionary(
    const std::string& code, ReferenceCount reference_count,
    std::unique_ptr<FileResolver> file_resolver)
    : L_(luaL_newstate()), //luaL_newstate源自lua5.2，起作用就像是malloc作用一样，用于初始化指针及给定内存大小
      index_into_reference_table_(-1),
      file_resolver_(std::move(file_resolver)),
      reference_count_(reference_count) {
  
  CHECK_NOTNULL(L_); //glog函数，用于检查L_是否为空
  //在注册表中设定好词典
  SetDictionaryInRegistry(L_, this);
  
  // lua: void luaL_openlibs (lua_State *L);
  // 打开指定状态机中的所有lua标准库
  luaL_openlibs(L_);

  // lua: void lua_register (lua_State *L, const char *name, lua_CFunction f);
  // 把C函数f设到全局变量name中,更简单一点的意思就是下列函数lua可以直接使用
  lua_register(L_, "choose", LuaChoose);
  // 将LuaInclude注册为Lua的全局函数变量,使得Lua可以调用C函数
  lua_register(L_, "include", LuaInclude);
  lua_register(L_, "read", LuaRead);
  // lua: int luaL_loadstring (lua_State *L, const char *s);
  // 将一个字符串加载为 Lua 代码块。 这个函数使用 lua_load 加载一个零结尾的字符串 s。
  // 此函数的返回值和 lua_load 相同。也和 lua_load 一样，这个函数仅加载代码块不运行。
  // NOTE: 这个地方将lua配置文件读入
  CheckForLuaErrors(L_, luaL_loadstring(L_, code.c_str()));
  // lua: void lua_call (lua_State *L, int nargs, int nresults);
  // 要调用一个函数请遵循以下协议： 首先，要调用的函数应该被压入栈； 接着，把需要传递给这个函数的参数按正序压栈；
  // 这是指第一个参数首先压栈。 最后调用一下 lua_call； nargs 是你压入栈的参数个数。
  // 当函数调用完毕后，所有的参数以及函数本身都会出栈。 而函数的返回值这时则被压栈。 返回值的个数将被调整为 nresults 个， 除非 nresults 被设置成 LUA_MULTRET。
  // 在这种情况下，所有的返回值都被压入堆栈中。 Lua 会保证返回值都放入栈空间中。 函数返回值将按正序压栈（第一个返回值首先压栈）， 因此在调用结束后，最后一个返回值将被放在栈顶。
  // lua: int lua_pcall (lua_State *L, int nargs, int nresults, int msgh);
  // 与lua_call一样，只不过更加安全，发生错误时返回错误码
  CheckForLuaErrors(L_, lua_pcall(L_, 0, 1, 0));
  CheckTableIsAtTopOfStack(L_);
}

LuaParameterDictionary::LuaParameterDictionary(
    lua_State* const L, ReferenceCount reference_count,
    std::shared_ptr<FileResolver> file_resolver)
    : L_(lua_newthread(L)),
      file_resolver_(std::move(file_resolver)),
      reference_count_(reference_count) {
  CHECK_NOTNULL(L_);

  // Make sure this is never garbage collected.
  CHECK(lua_isthread(L, -1));
  index_into_reference_table_ = luaL_ref(L, LUA_REGISTRYINDEX);

  CHECK(lua_istable(L, -1)) << "Topmost item on Lua stack is not a table!";
  lua_xmove(L, L_, 1);  // Moves the table and the coroutine over.
  CheckTableIsAtTopOfStack(L_);
}

LuaParameterDictionary::~LuaParameterDictionary() {
  if (reference_count_ == ReferenceCount::YES) {
    CheckAllKeysWereUsedExactlyOnceAndReset();
  }
  if (index_into_reference_table_ > 0) {
    luaL_unref(L_, LUA_REGISTRYINDEX, index_into_reference_table_);
  } else {
    lua_close(L_);
  }
}

std::vector<std::string> LuaParameterDictionary::GetKeys() const {
  CheckTableIsAtTopOfStack(L_);
  std::vector<std::string> keys;

  lua_pushnil(L_);  // Push the first key
  while (lua_next(L_, -2) != 0) {
    lua_pop(L_, 1);  // Pop value, keep key.
    if (!lua_isnumber(L_, -1)) {
      keys.emplace_back(lua_tostring(L_, -1));
    }
  }
  return keys;
}

bool LuaParameterDictionary::HasKey(const std::string& key) const {
  return HasKeyOfType(L_, key);
} 

// 获取key对应的string值
std::string LuaParameterDictionary::GetString(const std::string& key) {
  CheckHasKeyAndReference(key);
  GetValueFromLuaTable(L_, key);
  return PopString(Quoted::NO);
}

std::string LuaParameterDictionary::PopString(Quoted quoted) const {
  CHECK(lua_isstring(L_, -1)) << "Top of stack is not a string value.";
  if (quoted == Quoted::YES) {
    QuoteStringOnStack(L_);
  }

  const std::string value = lua_tostring(L_, -1);
  lua_pop(L_, 1);
  return value;
}

// 获取key对应的double值
double LuaParameterDictionary::GetDouble(const std::string& key) {
  CheckHasKeyAndReference(key);
  GetValueFromLuaTable(L_, key);
  return PopDouble();
}

double LuaParameterDictionary::PopDouble() const {
  CHECK(lua_isnumber(L_, -1)) << "Top of stack is not a number value.";
  const double value = lua_tonumber(L_, -1);
  lua_pop(L_, 1);
  return value;
}

// 获取key对应的int值
int LuaParameterDictionary::GetInt(const std::string& key) {
  CheckHasKeyAndReference(key);
  GetValueFromLuaTable(L_, key);
  return PopInt();
}

int LuaParameterDictionary::PopInt() const {
  CHECK(lua_isnumber(L_, -1)) << "Top of stack is not a number value.";
  const int value = lua_tointeger(L_, -1);
  lua_pop(L_, 1);
  return value;
}

// 获取key对应的bool值
bool LuaParameterDictionary::GetBool(const std::string& key) {
  CheckHasKeyAndReference(key);
  GetValueFromLuaTable(L_, key);
  return PopBool();
}

bool LuaParameterDictionary::PopBool() const {
  CHECK(lua_isboolean(L_, -1)) << "Top of stack is not a boolean value.";
  const bool value = lua_toboolean(L_, -1);
  lua_pop(L_, 1);
  return value;
}

std::unique_ptr<LuaParameterDictionary> LuaParameterDictionary::GetDictionary(
    const std::string& key) {
  CheckHasKeyAndReference(key);
  GetValueFromLuaTable(L_, key);
  return PopDictionary(reference_count_);
}

std::unique_ptr<LuaParameterDictionary> LuaParameterDictionary::PopDictionary(
    ReferenceCount reference_count) const {
  CheckTableIsAtTopOfStack(L_);
  std::unique_ptr<LuaParameterDictionary> value(
      new LuaParameterDictionary(L_, reference_count, file_resolver_));
  // The constructor lua_xmove()s the value, no need to pop it.
  CheckTableIsAtTopOfStack(L_);
  return value;
}

std::string LuaParameterDictionary::DoToString(
    const std::string& indent) const {
  std::string result = "{";
  bool dictionary_is_empty = true;

  const auto top_of_stack_to_string = [this, indent,
                                       &dictionary_is_empty]() -> std::string {
    dictionary_is_empty = false;

    const int value_type = lua_type(L_, -1);
    switch (value_type) {
      case LUA_TBOOLEAN:
        return PopBool() ? "true" : "false";
        break;
      case LUA_TSTRING:
        return PopString(Quoted::YES);
        break;
      case LUA_TNUMBER: {
        const double value = PopDouble();
        if (std::isinf(value)) {
          return value < 0 ? "-math.huge" : "math.huge";
        } else {
          return std::to_string(value);
        }
      } break;
      case LUA_TTABLE: {
        std::unique_ptr<LuaParameterDictionary> subdict(
            PopDictionary(ReferenceCount::NO));
        return subdict->DoToString(indent + "  ");
      } break;
      default:
        LOG(FATAL) << "Unhandled type " << lua_typename(L_, value_type);
    }
  };

  // Integer (array) keys.
  for (int i = 1; i; ++i) {
    GetValueFromLuaTable(L_, i);
    if (lua_isnil(L_, -1)) {
      lua_pop(L_, 1);
      break;
    }
    result.append("\n");
    result.append(indent);
    result.append("  ");
    result.append(top_of_stack_to_string());
    result.append(",");
  }

  // String keys.
  std::vector<std::string> keys = GetKeys();
  if (!keys.empty()) {
    std::sort(keys.begin(), keys.end());
    for (const std::string& key : keys) {
      GetValueFromLuaTable(L_, key);
      result.append("\n");
      result.append(indent);
      result.append("  ");
      result.append(key);
      result.append(" = ");
      result.append(top_of_stack_to_string());
      result.append(",");
    }
  }
  result.append("\n");
  result.append(indent);
  result.append("}");

  if (dictionary_is_empty) {
    return "{}";
  }
  return result;
}

std::string LuaParameterDictionary::ToString() const { return DoToString(""); }

std::vector<double> LuaParameterDictionary::GetArrayValuesAsDoubles() {
  std::vector<double> values;
  GetArrayValues(L_, [&values, this] { values.push_back(PopDouble()); });
  return values;
}

std::vector<std::unique_ptr<LuaParameterDictionary>>
LuaParameterDictionary::GetArrayValuesAsDictionaries() {
  std::vector<std::unique_ptr<LuaParameterDictionary>> values;
  GetArrayValues(L_, [&values, this] {
    values.push_back(PopDictionary(reference_count_));
  });
  return values;
}

std::vector<std::string> LuaParameterDictionary::GetArrayValuesAsStrings() {
  std::vector<std::string> values;
  GetArrayValues(L_,
                 [&values, this] { values.push_back(PopString(Quoted::NO)); });
  return values;
}

void LuaParameterDictionary::CheckHasKey(const std::string& key) const {
  CHECK(HasKey(key)) << "Key '" << key << "' not in dictionary:\n"
                     << ToString();
}

void LuaParameterDictionary::CheckHasKeyAndReference(const std::string& key) {
  CheckHasKey(key);
  reference_counts_[key]++;
}

void LuaParameterDictionary::CheckAllKeysWereUsedExactlyOnceAndReset() {
  for (const auto& key : GetKeys()) {
    CHECK_EQ(1, reference_counts_.count(key))
        << "Key '" << key << "' was used the wrong number of times.";
    CHECK_EQ(1, reference_counts_.at(key))
        << "Key '" << key << "' was used the wrong number of times.";
  }
  reference_counts_.clear();
}

int LuaParameterDictionary::GetNonNegativeInt(const std::string& key) {
  const int temp = GetInt(key);  // Will increase reference count.
  CHECK_GE(temp, 0) << temp << " is negative.";
  return temp;
}

// Lua function to run a script in the current Lua context. Takes the filename
// as its only argument.
// 读取.lua文件中的所有 include 的文件
int LuaParameterDictionary::LuaInclude(lua_State* L) {
  CHECK_EQ(lua_gettop(L), 1);
  CHECK(lua_isstring(L, -1)) << "include takes a filename.";

  LuaParameterDictionary* parameter_dictionary = GetDictionaryFromRegistry(L);
  //QUES: 这里的basename是咋获得的？？？，没看着往lua里传？？？
  //QUES: 为啥这里还调用了好几遍？？？
  const std::string basename = lua_tostring(L, -1);
  const std::string filename =
      parameter_dictionary->file_resolver_->GetFullPathOrDie(basename);
  // 防止双重包含
  if (std::find(parameter_dictionary->included_files_.begin(),
                parameter_dictionary->included_files_.end(),
                filename) != parameter_dictionary->included_files_.end()) {
    std::string error_msg =
        "Tried to include " + filename +
        " twice. Already included files in order of inclusion: ";
    for (const std::string& filename : parameter_dictionary->included_files_) {
      error_msg.append(filename);
      error_msg.append("\n");
    }
    LOG(FATAL) << error_msg;
  }
  parameter_dictionary->included_files_.push_back(filename);
  lua_pop(L, 1);
  CHECK_EQ(lua_gettop(L), 0);

  // 判断了2次文件是否存在
  const std::string content =
      parameter_dictionary->file_resolver_->GetFileContentOrDie(basename);
  //将map_builder.lua文件就放到了L中
  // lua: int luaL_loadbufferx (lua_State *L,
  // const char *buff,
  // size_t sz,
  // const char *name,
  // const char *mode);
  // 把一段缓存加载为一个 Lua 代码块。 这个函数使用 lua_load 来加载 buff 指向的长度为 sz 的内存区。
  
  //检查内存加载、函数调用是否会报错
  CheckForLuaErrors(
      L, luaL_loadbuffer(L, content.c_str(), content.size(), filename.c_str()));
  CheckForLuaErrors(L, lua_pcall(L, 0, LUA_MULTRET, 0));

  return lua_gettop(L);
}

// Lua function to read a file into a string.
int LuaParameterDictionary::LuaRead(lua_State* L) {
  CHECK_EQ(lua_gettop(L), 1);
  CHECK(lua_isstring(L, -1)) << "read takes a filename.";

  LuaParameterDictionary* parameter_dictionary = GetDictionaryFromRegistry(L);
  const std::string file_content =
      parameter_dictionary->file_resolver_->GetFileContentOrDie(
          lua_tostring(L, -1));
  lua_pushstring(L, file_content.c_str());
  return 1;
}

}  // namespace common
}  // namespace cartographer
