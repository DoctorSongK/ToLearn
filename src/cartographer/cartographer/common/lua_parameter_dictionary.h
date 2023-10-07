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

#ifndef CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_H_
#define CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/lua.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

// Resolves file paths and file content for the Lua 'read' and 'include'
// functions. Use this to configure where those functions load other files from.
/**
 * @brief 应用缓存流迭代器完成（lua-launch或install）配置文件的读取
 * 
 */
class FileResolver {
 public:
  virtual ~FileResolver() {}
  virtual std::string GetFullPathOrDie(const std::string& basename) = 0;
  virtual std::string GetFileContentOrDie(const std::string& basename) = 0;
};

// A parameter dictionary that gets loaded from Lua code.
/**
 * @brief 构建lua字典
 * 
 */
class LuaParameterDictionary {
 public:
  // Constructs the dictionary from a Lua Table specification.
  LuaParameterDictionary(const std::string& code,
                         std::unique_ptr<FileResolver> file_resolver);

  LuaParameterDictionary(const LuaParameterDictionary&) = delete;
  LuaParameterDictionary& operator=(const LuaParameterDictionary&) = delete;

  // Constructs a LuaParameterDictionary without reference counting.
  static std::unique_ptr<LuaParameterDictionary> NonReferenceCounted(
      const std::string& code, std::unique_ptr<FileResolver> file_resolver);

  ~LuaParameterDictionary();

  // Returns all available keys.
  std::vector<std::string> GetKeys() const;

  // Returns true if the key is in this dictionary.
  // @brief 检查是否有这个key
  bool HasKey(const std::string& key) const;

  // These methods CHECK() that the 'key' exists.
  std::string GetString(const std::string& key);
  double GetDouble(const std::string& key);
  int GetInt(const std::string& key);
  bool GetBool(const std::string& key);
  // @brief 返回字典中的某一个小字典组成
  std::unique_ptr<LuaParameterDictionary> GetDictionary(const std::string& key);

  // Gets an int from the dictionary and CHECK()s that it is non-negative.
  // 得到字典中key对应的value，并检查是否为非负值
  int GetNonNegativeInt(const std::string& key);

  // Returns a string representation for this LuaParameterDictionary.
  // 返回字典的字符串表示形式
  std::string ToString() const;

  // Returns the values of the keys '1', '2', '3' as the given types.
  std::vector<double> GetArrayValuesAsDoubles();
  std::vector<std::string> GetArrayValuesAsStrings();
  std::vector<std::unique_ptr<LuaParameterDictionary>>
  GetArrayValuesAsDictionaries();

 private:
  enum class ReferenceCount { YES, NO };
  /**
   * @brief 一种私有构造函数，具体使用还未知
   * 
   * @param[in] code 
   * @param[in] reference_count 
   * @param[in] file_resolver 
   */
  LuaParameterDictionary(const std::string& code,
                         ReferenceCount reference_count,
                         std::unique_ptr<FileResolver> file_resolver);

  // For GetDictionary().
  LuaParameterDictionary(lua_State* L, ReferenceCount reference_count,
                         std::shared_ptr<FileResolver> file_resolver);

  // Function that recurses to keep track of indent for ToString().
  std::string DoToString(const std::string& indent) const;

  // Pop the top of the stack and CHECKs that the type is correct.
  // pop出栈顶并检查这个类型是否正确
  double PopDouble() const;
  int PopInt() const;
  bool PopBool() const;

  // Pop the top of the stack and CHECKs that it is a string. The returned value
  // is either quoted to be suitable to be read back by a Lua interpretor or
  // not.
  enum class Quoted { YES, NO };
  std::string PopString(Quoted quoted) const;

  // Creates a LuaParameterDictionary from the Lua table at the top of the
  // stack, either with or without reference counting.
  // 
  std::unique_ptr<LuaParameterDictionary> PopDictionary(
      ReferenceCount reference_count) const;

  // CHECK() that 'key' is in the dictionary.
  // @brief 检查这个key是否在lua字典中
  void CheckHasKey(const std::string& key) const;

  // CHECK() that 'key' is in this dictionary and reference（引用） it as being used.
  void CheckHasKeyAndReference(const std::string& key);

  // If desired, this can be called in the destructor of a derived class. It
  // will CHECK() that all keys defined in the configuration have been used
  // exactly once and resets the reference counter.
  void CheckAllKeysWereUsedExactlyOnceAndReset();

  // Reads a file into a Lua string.
  /**
   * @brief 将文件读取为lua字符串
   * 
   * @param[in] L 
   * @return int 
   */
  static int LuaRead(lua_State* L);

  // Handles inclusion of other Lua files and prevents double inclusion.
  /**
   * @brief 处理其他lua文件的包含，并防止双重包含
   * 
   * @param[in] L 
   * @return int 
   */
  static int LuaInclude(lua_State* L);

  lua_State* L_;  // The name is by convention in the Lua World.
  int index_into_reference_table_;

  // This is shared with all the sub dictionaries.
  // @brief 所有子目录共享文件分解器
  const std::shared_ptr<FileResolver> file_resolver_;

  // If true will check that all keys were used on destruction.
  // @brief 若为true, 则检查所有key是否已在销毁时使用
  const ReferenceCount reference_count_;

  // This is modified with every call to Get* in order to verify that all
  // parameters are read exactly once.
  // @brief 每次调用get时都会对其进行修改，以验证所有参数是否只读取一次
  std::map<std::string, int> reference_counts_;

  // List of all included files in order of inclusion. Used to prevent double
  // inclusion.
  // 按包含顺序列出所有包含的文件。 用于防止双重包含
  std::vector<std::string> included_files_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_H_
