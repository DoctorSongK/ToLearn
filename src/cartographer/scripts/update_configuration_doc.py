#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""A dumb configuration.rst generator that relies on source comments."""

import io
import os

TARGET = 'docs/source/configuration.rst'
ROOT = 'cartographer'
PREFIX = """.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

=============
Configuration
=============

.. DO NOT EDIT! This documentation is AUTOGENERATED, please edit .proto files as
.. needed and run scripts/update_configuration_doc.py.

"""
SUFFIX = """
"""
NODOC = 'Not yet documented.'


class Message(object):
  def __init__(self, name, package, preceding_comments):
    self.name = name
    self.package = package
    self.preceding_comments = preceding_comments
    self.trailing_comments = None
    self.options = []

  def AddTrailingComments(self, comments):
    self.trailing_comments = comments

  def AddOption(self, option_type, name, comments):
    self.options.append((option_type, name, comments))


def ParseProtoFile(proto_file):
  """Computes the list of Message objects of the option messages in a file."""
  line_iter = iter(proto_file)

  # We ignore the license header and search for the 'package' line.
  for line in line_iter:
    line = line.strip()
    if line.startswith('package'):
      assert line[-1] == ';'
      package = line[7:-1].strip()
      break
    else:
      assert '}' not in line

  message_list = []
  while True:
    # Search for the next options message and capture preceding comments.
    message_comments = []
    for line in line_iter:
      line = line.strip()
      if '}' in line:
        # The preceding comments were for a different message it seems.
        message_comments = []
      elif line.startswith('//'):
        # We keep comments preceding an options message.
        comment = line[2:].strip()
        if not comment.startswith('NEXT ID:'):
          message_comments.append(comment)
      elif line.startswith('message') and line.endswith('Options {'):
        message_name = package + '.' + line[7:-1].strip()
        break
    else:
      # We reached the end of file.
      break
    print(" Found '%s'." % message_name)
    message = Message(message_name, package, message_comments)
    message_list.append(message)

    # We capture the contents of this message.
    option_comments = []
    multiline = ''
    for line in line_iter:
      line = line.strip()
      if '}' in line:
        # We reached the end of this message.
        message.AddTrailingComments(option_comments)
        break
      elif line.startswith('//'):
        comment = line[2:].strip()
        if not comment.startswith('NEXT ID:'):
          option_comments.append(comment)
      else:
        assert not line.startswith('required')
        multiline += ' ' + line
        if not multiline.endswith(';'):
          continue
        assert len(multiline) < 200
        option = multiline[:-1].strip().rstrip('0123456789').strip()
        assert option.endswith('=')
        if option.startswith('repeated'):
          option = option[8:]
        option_type, option_name = option[:-1].strip().split();
        print("  Option '%s'." % option_name)
        multiline = ''
        message.AddOption(option_type, option_name, option_comments)
        option_comments = []

  return message_list


def ParseProtoFilesRecursively(root):
  """Recursively parses all proto files into a list of Message objects."""
  message_list = []
  for dirpath, dirnames, filenames in os.walk(root):
    for name in filenames:
      if name.endswith('.proto'):
        path = os.path.join(dirpath, name)
        print("Found '%s'..." % path)
        assert not os.path.islink(path)
        message_list.extend(ParseProtoFile(io.open(path, encoding='UTF-8')))
  return message_list


class ResolutionError(Exception):
  """Raised when resolving a message name fails."""


class Resolver(object):
  def __init__(self, name_set):
    self.name_set = set(iter(name_set))

  def Resolve(self, message_name, package_name):
    if message_name in ('bool', 'double', 'float', 'int32'):
      return message_name
    if message_name.startswith('.'):
      return message_name[1:]
    package = package_name.split('.')
    for levels in range(len(package), -1, -1):
      candidate = '.'.join(package[0:levels]) + '.' + message_name
      if candidate in self.name_set:
        return candidate
    raise ResolutionError(
        'Resolving %s in %s failed.' % (message_name, package_name))


def GenerateDocumentation(output_file, root):
  """Recursively generates documentation, sorts and writes it."""
  message_list = ParseProtoFilesRecursively(root)
  resolver = Resolver(message.name for message in message_list)

  output_dict = {}
  for message in message_list:
    content = [message.name, '=' * len(message.name), '']
    assert message.name not in output_dict
    output_dict[message.name] = content
    if message.preceding_comments:
      content.extend(message.preceding_comments)
      content.append('')
    for option_type, option_name, option_comments in message.options:
      # TODO(whess): For now we exclude InitialTrajectoryPose from the
      # documentation. It is documented itself (since it has no Options suffix)
      # and is not parsed from the Lua files.
      if option_type in ('InitialTrajectoryPose',):
        continue
      content.append(
          resolver.Resolve(option_type, message.package) + ' ' + option_name)
      if not option_comments:
        option_comments.append(NODOC)
      for comment in option_comments:
        content.append('  ' + comment)
      content.append('')
    if message.trailing_comments:
      content.extend(message.trailing_comments)
      content.append('')

  output = ['\n'.join(doc) for key, doc in sorted(list(output_dict.items()))]
  print('\n\n'.join(output), file=output_file)


def main():
  assert not os.path.islink(TARGET) and os.path.isfile(TARGET)
  assert not os.path.islink(ROOT) and os.path.isdir(ROOT)
  output_file = io.open(TARGET, mode='w', encoding='UTF-8', newline='\n')
  output_file.write(PREFIX)
  GenerateDocumentation(output_file, ROOT)
  output_file.write(SUFFIX)
  output_file.close()


if __name__ == "__main__":
  main()
