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

#include "cartographer/common/configuration_file_resolver.h"

#include <fstream>
#include <iostream>
#include <streambuf>

#include "cartographer/common/config.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

/**
 * @brief 构造lua文件解析对象，这里将install、launch输入lua文件都放入至容器中，且launch中配置优先。
 * 
 * @param[in] configuration_files_directories 配置文件目录，包含两个文件目录，一个是从launch中读到的，一个是从install中读到的配置
 */
ConfigurationFileResolver::ConfigurationFileResolver(
    const std::vector<std::string>& configuration_files_directories)
    : configuration_files_directories_(configuration_files_directories) {
  //这里的kConfigurationFilesDirectory是编译时生成的文件绝对地址
  configuration_files_directories_.push_back(kConfigurationFilesDirectory);
}

/**
 * @brief 在所有的配置文件目录中 根据给定配置文件的名字 搜索 配置文件
 * 
 * @param[in] basename 给定配置文件的名字
 * @return std::string 如果搜索成功, 返回配置文件的全路径名
 */
std::string ConfigurationFileResolver::GetFullPathOrDie(
    const std::string& basename) {
  for (const auto& path : configuration_files_directories_) {
    const std::string filename = path + "/" + basename;
    std::ifstream stream(filename.c_str());
    // 只要找到就退出，.good()是成功打开文件夹
    if (stream.good()) {
      LOG(INFO) << "Found '" << filename << "' for '" << basename << "'.";
      return filename;
    }
  }
  // 如果找不到配置文件就退出整个程序
  LOG(FATAL) << "File '" << basename << "' was not found.";
}

/**
 * @brief 读取配置文件内容
 * 
 * @param[in] basename 文件名
 * @return std::string 文件内容的数据流
 */
std::string ConfigurationFileResolver::GetFileContentOrDie(
    const std::string& basename) {
  CHECK(!basename.empty()) << "File basename cannot be empty." << basename;

  // 根据文件名查找是否在给定文件夹中存在
  const std::string filename = GetFullPathOrDie(basename);
  std::ifstream stream(filename.c_str());

  // 读取配置文件内容并返回
  // c++11: 应用std::istreambuf_iterator 完成整个文本文件转换至string中，该方式运行效率高，直接读取缓冲区
  return std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
}

}  // namespace common
}  // namespace cartographer
