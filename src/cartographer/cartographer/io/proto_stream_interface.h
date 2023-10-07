/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_IO_PROTO_STREAM_INTERFACE_H_
#define CARTOGRAPHER_IO_PROTO_STREAM_INTERFACE_H_

#include "cartographer/common/port.h"
#include "google/protobuf/message.h"

namespace cartographer {
namespace io {

// A writer for writing proto messages to a pbstream.
// 用于将proto消息写入到pbstream中的接口类
class ProtoStreamWriterInterface {
 public:
  virtual ~ProtoStreamWriterInterface() {}

  // Serializes, compressed and writes the 'proto' to the file.
  // 序列化操作，压缩proto信息并将其写入到文件中
  virtual void WriteProto(const google::protobuf::Message& proto) = 0;

  // This should be called to check whether writing was successful.
  // 调用此操作来检查写入是否成功
  virtual bool Close() = 0;
};

// A reader of the format produced by ProtoStreamWriter.
// 读取pbstream格式的接口类
class ProtoStreamReaderInterface {
 public:
  ProtoStreamReaderInterface() = default;
  virtual ~ProtoStreamReaderInterface() {}

  // Deserialize compressed proto from the pb stream.
  // 反序列化操作，将pbstream压缩文件提取到proto中
  virtual bool ReadProto(google::protobuf::Message* proto) = 0;

  // 'End-of-file' marker for the pb stream.
  virtual bool eof() const = 0;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PROTO_STREAM_INTERFACE_H_
