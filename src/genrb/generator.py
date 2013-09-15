#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Takashi Ogura
#
# based on
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id:$

"""
Library for Ruby message generation.

The structure of the serialization descends several levels of serializers:
 - msg_generator: generator for an individual msg file
  - serialize_fn_generator: generator for msg.serialize()
    - serializer_generator
      - field-type-specific serializers
  - deserialize_fn_generator: generator for msg.deserialize()
    - serializer_generator
      - field-type-specific serializers
"""

from __future__ import print_function

import os
import shutil
import atexit
import itertools
import sys
import tempfile
import traceback
import struct

try:
    from cStringIO import StringIO # Python 2.x
except ImportError:
    from io import StringIO # Python 3.x

import genmsg
import genmsg.msgs
import genmsg.msg_loader
import genmsg.gentools

from genmsg import InvalidMsgSpec, MsgContext, MsgSpec, MsgGenerationException


# indent width
INDENT = '  '

def get_registered_ex(msg_context, type_):
    """
    wrapper for genmsg.msgs.get_registered that wraps unknown types with a MsgGenerationException
    @param type_: ROS message type
    @type  type_: str
    """
    try:
        return msg_context.get_registered(type_)
    except KeyError:
        raise MsgGenerationException("Unknown type [%s]. Please check that the manifest.xml correctly declares dependencies."%type_)

################################################################################
# Primitive type handling for ROS builtin types

PYTHON_RUBY_DICT = { #see python module struct
    'b': 'c',
    'B': 'C',
    'h' : 's',
    'H' : 'S',
    'i' : 'l',
    'I' : 'L',
    'q' : 'q',
    'Q' : 'Q',
    'f': 'f',
    'd': 'd'
    }

SIMPLE_TYPES_DICT = { #see python module struct
    'int8': 'b',
    'uint8': 'B',
    # Python 2.6 adds in '?' for C99 _Bool, which appears equivalent to an uint8,
    # thus, we use uint8
    'bool': 'B',
    'int16' : 'h',
    'uint16' : 'H',
    'int32' : 'i',
    'uint32' : 'I',
    'int64' : 'q',
    'uint64' : 'Q',
    'float32': 'f',
    'float64': 'd',
    # deprecated
    'char' : 'B', #unsigned
    'byte' : 'b', #signed
    }

## Simple types are primitives with fixed-serialization length
SIMPLE_TYPES = list(SIMPLE_TYPES_DICT.keys()) #py3k

def is_simple(type_):
    """
    @return bool: True if type is a 'simple' type, i.e. is of
    fixed/known serialization length. This is effectively all primitive
    types except for string
    @rtype: bool
    """
    return type_ in SIMPLE_TYPES

def is_special(type_):
    """
    @return True if type_ is a special type (i.e. builtin represented as a class instead of a primitive)
    @rtype: bool
    """
    return type_ in _SPECIAL_TYPES

def get_special(type_):
    """
    @return: special type handler for type_ or None
    @rtype: L{Special}
    """
    return _SPECIAL_TYPES.get(type_, None)

################################################################################
# Special type handling for ROS builtin types that are not primitives

class Special:

    def __init__(self, constructor, post_deserialize, import_str):
        """
        @param constructor: expression to instantiate new type instance for deserialization
        @type  constructor: str
        @param post_Deserialize: format string for expression to evaluate on type instance after deserialization is complete.
        @type  post_Deserialize: str
          variable name will be passed in as the single argument to format string.
        @param import_str: import to include if type is present
        @type  import_str: str
        """
        self.constructor = constructor
        self.post_deserialize = post_deserialize
        self.import_str = import_str

    def get_post_deserialize(self, varname):
        """
        @return: Post-deserialization code to executed (unindented) or
        None if no post-deserialization is required
        @rtype: str
        """
        if self.post_deserialize:
            return self.post_deserialize%varname
        else:
            return None

_SPECIAL_TYPES = {
    genmsg.HEADER:   Special('Std_msgs::Header.new',     None, 'require "std_msgs/Header"'),
    genmsg.TIME:     Special('ROS::Time.new',     None, 'require "ros/time"'),
    genmsg.DURATION: Special('ROS::Duration.new', None, 'require "ros/duration"'),
    }

################################################################################
# utilities

# #671
def default_value(msg_context, field_type, default_package):
    """
    Compute default value for field_type
    @param default_package: default package
    @type  default_package: str
    @param field_type str: ROS .msg field type
    @type  field_type: ROS .msg field type
    @return: default value encoded in Python string representation
    @rtype: str
    """
    if field_type in ['byte', 'int8', 'int16', 'int32', 'int64',\
                          'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif field_type in ['float32', 'float64']:
        return '0.0'
    elif field_type == 'string':
        # strings, char[], and uint8s are all optimized to be strings
        return "''"
    elif field_type == 'bool':
        return 'false'
    elif field_type.endswith(']'): # array type
        base_type, is_array, array_len = genmsg.msgs.parse_type(field_type)
        if base_type in ['char', 'uint8']:
            # strings, char[], and uint8s are all optimized to be strings
            if array_len is not None:
                return "[0].pack('c')*%s"%array_len
            else:
                return "''"
        elif array_len is None: #var-length
            return '[]'
        else: # fixed-length, fill values
            def_val = default_value(msg_context, base_type, default_package)
            return '[' + ','.join(itertools.repeat(def_val, array_len)) + ']'
    else:
        return compute_constructor(msg_context, default_package, field_type)

def flatten(msg_context, msg):
    """
    Flattens the msg spec so that embedded message fields become
    direct references. The resulting MsgSpec isn't a true/legal
    L{MsgSpec} and should only be used for serializer generation
    @param msg: msg to flatten
    @type  msg: L{MsgSpec}
    @return: flatten message
    @rtype: L{MsgSpec}
    """
    new_types = []
    new_names = []
    for t, n in zip(msg.types, msg.names):
        #flatten embedded types - note: bug #59
        msg_type, is_array, _ = genmsg.msgs.parse_type(t)
        if not is_array and msg_context.is_registered(t):
            msg_spec = flatten(msg_context, msg_context.get_registered(t))
            new_types.extend(msg_spec.types)
            for n2 in msg_spec.names:
                new_names.append(n+'.'+n2)
        else:
            #I'm not sure if it's a performance win to flatten fixed-length arrays
            #as you get n __getitems__ method calls vs. a single *array call
            new_types.append(t)
            new_names.append(n)
    return MsgSpec(new_types, new_names, msg.constants, msg.text, msg.full_name)

def make_ruby_safe(spec):
    """
    Remap field/constant names in spec to avoid collision with Python reserved words.
    @param spec: msg spec to map to new, python-safe field names
    @type  spec: L{MsgSpec}
    @return: python-safe message specification
    @rtype: L{MsgSpec}
    """
    new_c = [genmsg.Constant(c.type, _remap_reserved(c.name), c.val, c.val_text) for c in spec.constants]
    return MsgSpec(spec.types, [_remap_reserved(n) for n in spec.names], new_c, spec.text, spec.full_name)

def _remap_reserved(field_name):
    """
    Map field_name to a python-safe representation, if necessary
    @param field_name: msg field name
    @type  field_name: str
    @return: remapped name
    @rtype: str
    """
    # include 'self' as well because we are within a class instance
    if field_name in ['BEGIN', 'END', 'alias', 'and', 'begin', 'break', 'case', 'class', 'def', 'defined?', 'do', 'else', 'elsif', 'end', 'ensure', 'false', 'for', 'if', 'in', 'module', 'next', 'nil', 'not', 'or', 'redo', 'rescue', 'retry', 'return', 'self', 'super', 'then', 'true', 'undef', 'unless', 'until', 'when', 'while', 'yield']:
        return field_name + "_"
    return field_name

################################################################################
# (de)serialization routines

def compute_struct_pattern(types):
    """
    @param types: type names
    @type  types: [str]
    @return: format string for struct if types are all simple. Otherwise, return None
    @rtype: str
    """
    if not types: #important to filter None and empty first
        return None
    try:
        return ''.join([SIMPLE_TYPES_DICT[t] for t in types])
    except:
        return None

def compute_post_deserialize(type_, varname):
    """
    Compute post-deserialization code for type_, if necessary
    @return: code to execute post-deserialization (unindented), or None if not necessary.
    @rtype: str
    """
    s = get_special(type_)
    if s is not None:
        return s.get_post_deserialize(varname)

def compute_constructor(msg_context, package, type_):
    """
    Compute python constructor expression for specified message type implementation
    @param package str: package that type is being imported into. Used
        to resolve type_ if package is not specified.
    @type  package: str
    @param type_: message type
    @type  type_: str
    """
    if is_special(type_):
        return get_special(type_).constructor
    else:
        base_pkg, base_type_ = compute_pkg_type(package, type_)
        if not msg_context.is_registered("%s/%s"%(base_pkg,base_type_)):
            return None
        else:
            return '%s::%s.new'%(base_pkg.capitalize(), base_type_)

def compute_pkg_type(package, type_):
    """
    @param package: package that type is being imported into
    @type  package: str
    @param type: message type (package resource name)
    @type  type: str
    @return (str, str): python package and type name
    """
    splits = type_.split(genmsg.SEP)
    if len(splits) == 1:
        return package, splits[0]
    elif len(splits) == 2:
        return tuple(splits)
    else:
        raise MsgGenerationException("illegal message type: %s"%type_)

def compute_import(msg_context, package, type_):
    """
    Compute python import statement for specified message type implementation
    @param package: package that type is being imported into
    @type  package: str
    @param type_: message type (package resource name)
    @type  type_: str
    @return: list of import statements (no newline) required to use type_ from package
    @rtype: [str]
    """
    # orig_base_type is the unresolved type
    orig_base_type = genmsg.msgs.bare_msg_type(type_) # strip array-suffix
    # resolve orig_base_type based on the current package context.
    # base_type is the resolved type stripped of any package name.
    # pkg is the actual package of type_.
    pkg, base_type = compute_pkg_type(package, orig_base_type)
    type_str = "%s/%s"%(pkg, base_type) # compute fully-qualified type
    # important: have to do is_builtin check first. We do this check
    # against the unresolved type builtins/specials are never
    # relative. This requires some special handling for Header, which has
    # two names (Header and std_msgs/Header).
    if genmsg.msgs.is_builtin(orig_base_type) or \
           genmsg.msgs.is_header_type(orig_base_type):
        # of the builtin types, only special types require import
        # handling. we switch to base_type as special types do not
        # include package names.
        if is_special(base_type):
            retval = [get_special(base_type).import_str]
        else:
            retval = []
    elif not msg_context.is_registered(type_str):
        retval = []
    else:
        retval = ['require "%s/%s"'%(pkg, base_type)]
        for t in get_registered_ex(msg_context, type_str).types:
            sub = compute_import(msg_context, package, t)
            retval.extend([x for x in sub if not x in retval])
    return retval

def compute_full_text_escaped(msg_context, spec):
    """
    Same as genmsg.compute_full_text, except that the
    resulting text is escaped to be safe for Python's triple-quote string
    quoting

    @param get_deps_dict: dictionary returned by get_dependencies call
    @type  get_deps_dict: dict
    @return: concatenated text for msg/srv file and embedded msg/srv types. Text will be escaped for triple-quote
    @rtype: str
    """
    msg_definition = genmsg.compute_full_text(msg_context, spec)
    msg_definition = msg_definition.replace('"', '\\"')
    return msg_definition

def reduce_pattern(pattern):
    """
    Optimize the struct format pattern.
    @param pattern: struct pattern
    @type  pattern: str
    @return: optimized struct pattern
    @rtype: str
    """
    if not pattern or len(pattern) == 1 or '%' in pattern:
        return pattern
    prev = pattern[0]
    count = 1
    new_pattern = ''
    nums = [str(i) for i in range(0, 9)]
    for c in pattern[1:]:
        if c == prev and not c in nums:
            count += 1
        else:
            if count > 1:
                new_pattern = new_pattern + str(count) + prev
            else:
                new_pattern = new_pattern + prev
            prev = c
            count = 1
    if count > 1:
        new_pattern = new_pattern + str(count) + c
    else:
        new_pattern = new_pattern + prev
    return new_pattern

## @param expr str: string python expression that is evaluated for serialization
## @return str: python call to write value returned by expr to serialization buffer
def serialize(expr):
    return "buff.write(%s)"%expr

# int32 is very common due to length serialization, so it is special cased
def int32_pack(var):
    """
    @param var: variable name
    @type  var: str
    @return: struct packing code for an int32
    """
    return serialize('@@struct_L.pack(%s)'%var)

# int32 is very common due to length serialization, so it is special cased
def int32_unpack(var, buff):
    """
    @param var: variable name
    @type  var: str
    @return: struct unpacking code for an int32
    """
    return '(%s,) = @@struct_L.unpack(%s)'%(var, buff)

#NOTE: '<' = little endian
def pack(pattern, vars):
    """
    create struct.pack call for when pattern is a string pattern
    @param pattern: pattern for pack
    @type  pattern: str
    @param vars: name of variables to pack
    @type  vars: str
    """
    # - store pattern in context
    pattern = reduce_pattern(pattern)
    add_pattern(pattern)
    return serialize("@@struct_%s.pack(%s)"%(convert_to_ruby_pattern(pattern),
                                            vars))
def pack2(pattern, vars):
    """
    create struct.pack call for when pattern is the name of a variable
    @param pattern: name of variable storing string pattern
    @type  pattern: struct
    @param vars: name of variables to pack
    @type  vars: str
    """
    return serialize("%s.pack(%s)"%(vars, pattern))
def unpack(var, pattern, buff):
    """
    create struct.unpack call for when pattern is a string pattern
    @param var: name of variable to unpack
    @type  var: str
    @param pattern: pattern for pack
    @type  pattern: str
    @param buff: buffer to unpack from
    @type  buff: str
    """
    # - store pattern in context
    pattern = reduce_pattern(pattern)
    add_pattern(pattern)
    return var + " = @@struct_%s.unpack(%s)"%(convert_to_ruby_pattern(pattern),
                                             buff)
def unpack2(var, pattern, buff):
    """
    Create struct.unpack call for when pattern refers to variable
    @param var: variable the stores the result of unpack call
    @type  var: str
    @param pattern: name of variable that unpack will read from
    @type  pattern: str
    @param buff: buffer that the unpack reads from
    @type  buff: StringIO
    """
    return "%s = %s.unpack(%s)"%(var, buff, pattern)

################################################################################
# numpy support

# this could obviously be directly generated, but it's nice to abstract

## maps ros msg types to numpy types
_NUMPY_DTYPE = {
    'float32': 'numpy.float32',
    'float64': 'numpy.float64',
    'bool': 'numpy.bool',
    'int8': 'numpy.int8',
    'int16': 'numpy.int16',
    'int32': 'numpy.int32',
    'int64': 'numpy.int64',
    'uint8': 'numpy.uint8',
    'uint16': 'numpy.uint16',
    'uint32': 'numpy.uint32',
    'uint64': 'numpy.uint64',
    # deprecated type
    'char' : 'numpy.uint8',
    'byte' : 'numpy.int8',
    }

################################################################################
# (De)serialization generators

_serial_context = ''
_context_stack = []

_counter = 0
def next_var():
    # we could optimize this by reusing vars once the context is popped
    global _counter
    _counter += 1
    return '_v%s'%_counter

def push_context(context):
    """
    Push new variable context onto context stack.  The context stack
    manages field-reference context for serialization, e.g. 'self.foo'
    vs. 'self.bar.foo' vs. 'var.foo'
    """
    global _serial_context, _context_stack
    _context_stack.append(_serial_context)
    _serial_context = context

def pop_context():
    """
    Pop variable context from context stack.  The context stack manages
    field-reference context for serialization, e.g. 'self.foo'
    vs. 'self.bar.foo' vs. 'var.foo'
    """
    global _serial_context
    _serial_context = _context_stack.pop()

_context_patterns = []
def add_pattern(p):
    """
    Record struct pattern that's been used for (de)serialization
    """
    _context_patterns.append(p)
def clear_patterns():
    """
    Clear record of struct pattern that have been used for (de)serialization
    """
    del _context_patterns[:]
def get_patterns():
    """
    @return: record of struct pattern that have been used for (de)serialization
    """
    return _context_patterns[:]

# These are the workhorses of the message generation. The generators
# are implemented as iterators, where each iteration value is a line
# of Python code. The generators will invoke underlying generators,
# using the context stack to manage any changes in variable-naming, so
# that code can be reused as much as possible.

def len_serializer_generator(var, is_string, serialize):
    """
    Generator for array-length serialization (32-bit, little-endian unsigned integer)
    @param var: variable name
    @type  var: str
    @param is_string: if True, variable is a string type
    @type  is_string: bool
    @param serialize bool: if True, generate code for
    serialization. Other, generate code for deserialization
    @type  serialize: bool
    """
    if serialize:
        yield "length = %s.length"%var
        # NOTE: it's more difficult to save a call to struct.pack with
        # the array length as we are already using *array_val to pass
        # into struct.pack as *args. Although it's possible that
        # Python is not optimizing it, it is potentially worse for
        # performance to attempt to combine
        if not is_string:
            yield int32_pack("length")
    else:
        yield "start = end_point"
        yield "end_point += 4"
        yield int32_unpack('length', 'str[start..(end_point-1)]') #4 = struct.calcsize('<i')

def string_serializer_generator(package, type_, name, serialize):
    """
    Generator for string types. similar to arrays, but with more
    efficient call to struct.pack.
    @param name: spec field name
    @type  name: str
    @param serialize: if True, generate code for
    serialization. Other, generate code for deserialization
    @type  serialize: bool
    """
    # don't optimize in deserialization case as assignment doesn't
    # work
    if _serial_context and serialize:
        # optimize as string serialization accesses field twice
        yield "_x = %s%s"%(_serial_context, name)
        var = "_x"
    else:
        var = _serial_context+name

    # the length generator is a noop if serialize is True as we
    # optimize the serialization call.
    base_type, is_array, array_len = genmsg.msgs.parse_type(type_)
    # - don't serialize length for fixed-length arrays of bytes
    if base_type not in ['uint8', 'char'] or array_len is None:
        for y in len_serializer_generator(var, True, serialize):
            yield y #serialize string length

    if serialize:
        #serialize length and string together

        #check to see if its a uint8/char type, in which case we need to convert to string before serializing
        base_type, is_array, array_len = genmsg.msgs.parse_type(type_)
        if base_type in ['uint8', 'char']:
#            yield "# - if encoded as a list instead, serialize as bytes instead of string"
            if array_len is None:
#                yield "if type(%s) in [list, tuple]"%var
#                yield INDENT+pack2('"La#{length}"', "[length, *%s]"%var)
#                yield "else"
                yield pack2('"La#{length}"', "[length, %s]"%var)
#                yield "end"
            else:
#                yield "if type(%s) in [list, tuple]"%var
#                yield INDENT+pack('C%s'%array_len, "*%s"%var)
#                yield "else"
                yield pack('C%s'%array_len, var)
#                yield "end"
        else:
            # py3k: struct.pack() now only allows bytes for the s string pack code.
            # FIXME: for py3k, this needs to be w/ encode, but this interferes with actual byte data
            #yield pack2("'<I%ss'%length", "length, %s.encode()"%var) #Py3k bugfix (see http://docs.python.org/dev/whatsnew/3.2.html#porting-to-python-3-2)
            yield pack2('"La#{length}"', "[length, %s]"%var)
    else:
        yield "start = end_point"
        if array_len is not None:
            yield "end_point += %s" % array_len
        else:
            yield "end_point += length"
        yield "%s = str[start..(end_point-1)]" % var

def array_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
    """
    Generator for array types
    @raise MsgGenerationException: if array spec is invalid
    """
    base_type, is_array, array_len = genmsg.msgs.parse_type(type_)
    if not is_array:
        raise MsgGenerationException("Invalid array spec: %s"%type_)
    var_length = array_len is None

    # handle fixed-size byte arrays could be slightly more efficient
    # as we recalculated the length in the generated code.
    if base_type in ['char', 'uint8']: #treat unsigned int8 arrays as string type
        for y in string_serializer_generator(package, type_, name, serialize):
            yield y
        return

    var = _serial_context+name
    try:
        # yield length serialization, if necessary
        if var_length:
            for y in len_serializer_generator(var, False, serialize):
                yield y #serialize array length
            length = None
        else:
            length = array_len

        #optimization for simple arrays
        if is_simple(base_type):
            if var_length:
                pattern = compute_struct_pattern([base_type])
                yield 'pattern = "%s#{length}"'%convert_to_ruby_pattern(pattern)
                if serialize:
                    if is_numpy:
                        yield pack_numpy(var)
                    else:
                        yield pack2('pattern', "*"+var)
                else:
                    yield "start = end_point"
                    yield 'end_point += ROS::Struct::calc_size("#{pattern}")'
                    if is_numpy:
                        dtype = _NUMPY_DTYPE[base_type]
                        yield unpack_numpy(var, 'length', dtype, 'str[start..(end_point-1)]')
                    else:
                        yield unpack2(var, 'pattern', 'str[start..(end_point-1)]')
            else:
                pattern = "%s%s"%(length, compute_struct_pattern([base_type]))
                if serialize:
                    if is_numpy:
                        yield pack_numpy(var)
                    else:
                        yield pack(pattern, "*"+var)
                else:
                    yield "start = end_point"
                    yield "end_point += ROS::Struct::calc_size('%s')"%convert_to_ruby_pattern(pattern)
                    if is_numpy:
                        dtype = _NUMPY_DTYPE[base_type]
                        yield unpack_numpy(var, length, dtype, 'str[start..(end_point-1)]')
                    else:
                        yield unpack(var, pattern, 'str[start..(end_point-1)]')
            if not serialize and base_type == 'bool':
                # convert uint8 to bool
                if base_type == 'bool':
                    yield "%s = map(bool, %s)"%(var, var)

        else:
            #generic recursive serializer
            #NOTE: this is functionally equivalent to the is_registered branch of complex_serializer_generator

            # choose a unique temporary variable for iterating
            loop_var = 'val%s'%len(_context_stack)

            # compute the variable context and factory to use
            if base_type == 'string':
                push_context('')
                factory = string_serializer_generator(package, base_type, loop_var, serialize)
            else:
                push_context('%s.'%loop_var)
                factory = serializer_generator(msg_context, get_registered_ex(msg_context, base_type), serialize, is_numpy)

            if serialize:
                yield 'for %s in %s'%(loop_var, var)
            else:
                yield '%s = []'%var
                if var_length:
                    yield 'length.times do'
                else:
                    yield '%s.times do'%length
                if base_type != 'string':
                    yield INDENT + '%s = %s'%(loop_var, compute_constructor(msg_context, package, base_type))
            for y in factory:
                yield INDENT + y
            if not serialize:
                yield INDENT + '%s.push(%s)'%(var, loop_var)
            pop_context()
            yield 'end'

    except MsgGenerationException:
        raise #re-raise
    except Exception as e:
        raise MsgGenerationException(e) #wrap

def complex_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
    """
    Generator for serializing complex type
    @param serialize: if True, generate serialization
    code. Otherwise, deserialization code.
    @type  serialize: bool
    @param is_numpy: if True, generate serializer code for numpy
    datatypes instead of Python lists
    @type  is_numpy: bool
    @raise MsgGenerationException: if type is not a valid
    """
    # ordering of these statements is important as we mutate the type
    # string we are checking throughout. parse_type strips array
    # brackets, then we check for the 'complex' builtin types (string,
    # time, duration, Header), then we canonicalize it to an embedded
    # message type.
    _, is_array, _ = genmsg.msgs.parse_type(type_)

    #Array
    if is_array:
        for y in array_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
            yield y
    #Embedded Message
    elif type_ == 'string':
        for y in string_serializer_generator(package, type_, name, serialize):
            yield y
    else:
        if not is_special(type_):
            # canonicalize type
            pkg, base_type = compute_pkg_type(package, type_)
            type_ = "%s/%s"%(pkg, base_type)
        if msg_context.is_registered(type_):
            # descend data structure ####################
            ctx_var = next_var()
            yield "%s = %s"%(ctx_var, _serial_context+name)
            push_context(ctx_var+'.')
            # unoptimized code
            #push_context(_serial_context+name+'.')
            for y in serializer_generator(msg_context, get_registered_ex(msg_context, type_), serialize, is_numpy):
                yield y #recurs on subtype
            pop_context()
        else:
            #Invalid
            raise MsgGenerationException("Unknown type: %s. Package context is %s"%(type_, package))

def simple_serializer_generator(msg_context, spec, start, end_point, serialize): #primitives that can be handled with struct
    """
    Generator (de)serialization code for multiple fields from spec
    @param spec: MsgSpec
    @type  spec: MsgSpec
    @param start: first field to serialize
    @type  start: int
    @param end: last field to serialize
    @type  end: int
    """
    # optimize member var access
    if end_point - start > 1 and _serial_context.endswith('.'):
        yield '_x = '+_serial_context[:-1]
        vars_ = '_x.' + (', _x.').join(spec.names[start:end_point])
    else:
        vars_ = _serial_context + (', '+_serial_context).join(spec.names[start:end_point])

    pattern = compute_struct_pattern(spec.types[start:end_point])
    if serialize:
        yield pack(pattern, vars_)
    else:
        yield "start = end_point"
        yield "end_point += ROS::Struct::calc_size('%s')"%convert_to_ruby_pattern(reduce_pattern(pattern))
        yield unpack('(%s,)'%vars_, pattern, 'str[start..(end_point-1)]')

        # convert uint8 to bool. this doesn't add much value as Python
        # equality test on a field will return that True == 1, but I
        # want to be consistent with bool
        bool_vars = [(f, t) for f, t in zip(spec.names[start:end_point], spec.types[start:end_point]) if t == 'bool']
        for f, t in bool_vars:
            #TODO: could optimize this as well
            var = _serial_context+f
            yield "%s = bool(%s)"%(var, var)

def serializer_generator(msg_context, spec, serialize, is_numpy):
    """
    Python generator that yields un-indented python code for
    (de)serializing MsgSpec. The code this yields is meant to be
    included in a class method and cannot be used
    standalone. serialize_fn_generator and deserialize_fn_generator
    wrap method to provide appropriate class field initializations.
    @param package: name of package the spec is being used in
    @type  package: str
    @param serialize: if True, yield serialization
    code. Otherwise, yield deserialization code.
    @type  serialize: bool
    @param is_numpy: if True, generate serializer code for numpy datatypes instead of Python lists
    @type  is_numpy: bool
    """
    # Break spec into chunks of simple (primitives) vs. complex (arrays, etc...)
    # Simple types are batch serialized using the python struct module.
    # Complex types are individually serialized
    if spec is None:
        raise MsgGenerationException("spec is none")
    names, types = spec.names, spec.types
    if serialize and not len(names): #Empty
        yield "pass"
        return

    # iterate through types. whenever we encounter a non-simple type,
    # yield serializer for any simple types we've encountered until
    # then, then yield the complex type serializer
    curr = 0
    for (i, full_type) in enumerate(types):
        if not is_simple(full_type):
            if i != curr: #yield chunk of simples
                for y in simple_serializer_generator(msg_context, spec, curr, i, serialize):
                    yield y
            curr = i+1
            for y in complex_serializer_generator(msg_context, spec.package, full_type, names[i], serialize, is_numpy):
                yield y
    if curr < len(types): #yield rest of simples
        for y in simple_serializer_generator(msg_context, spec, curr, len(types), serialize):
            yield y

def serialize_fn_generator(msg_context, spec, is_numpy=False):
    """
    generator for body of serialize() function
    @param is_numpy: if True, generate serializer code for numpy
    datatypes instead of Python lists
    @type  is_numpy: bool
    """
    # method-var context #########
    yield "begin"
    push_context('@')
    #NOTE: we flatten the spec for optimal serialization
    flattened = make_ruby_safe(flatten(msg_context, spec))
    for y in serializer_generator(msg_context, flattened, True, is_numpy):
        yield "  "+y
    pop_context()
    yield """rescue => exception
      raise "some erro in serialize: #{exception}"
"""
    yield "end"
    # done w/ method-var context #

def deserialize_fn_generator(msg_context, spec, is_numpy=False):
    """
    generator for body of deserialize() function
    @param is_numpy: if True, generate serializer code for numpy
    datatypes instead of Python lists
    @type  is_numpy: bool
    """
    yield "begin"

    #Instantiate embedded type classes
    for type_, name in spec.fields():
        if msg_context.is_registered(type_):
            yield "  if @%s == nil"%name
            yield "    @%s = %s"%(name, compute_constructor(msg_context, spec.package, type_))
            yield "  end"
    yield "  end_point = 0" #initialize var

    # method-var context #########
    push_context('@')
    #NOTE: we flatten the spec for optimal serialization
    flattened = make_ruby_safe(flatten(msg_context, spec))
    for y in serializer_generator(msg_context, flattened, False, is_numpy):
        yield "  "+y
    pop_context()
    # done w/ method-var context #

    # generate post-deserialization code
    for type_, name in spec.fields():
        code = compute_post_deserialize(type_, "@%s"%name)
        if code:
            yield "  %s"%code

    yield "  return self"
    yield "rescue => exception"
    yield """  raise \"message DeserializationError: #{exception}\" #most likely buffer underfill
    end"""

def msg_generator_internal(msg_context, spec, search_path):
    """
    Python code generator for .msg files. Takes in a package name,
    message name, and message specification and generates a Python
    message class.

    @param package: name of package for message
    @type  package: str
    @param name: base type name of message, e.g. 'Empty', 'String'
    @type  name: str
    @param spec: parsed .msg specification
    @type  spec: L{MsgSpec}
    """

    # #2990: have to compute md5sum before any calls to make_python_safe

    # generate dependencies dictionary. omit files calculation as we
    # rely on in-memory MsgSpecs instead so that we can generate code
    # for older versions of msg files
    try:
        genmsg.msg_loader.load_depends(msg_context, spec, search_path)
    except InvalidMsgSpec as e:
        raise MsgGenerationException("Cannot generate .msg for %s/%s: %s"%(package, name, str(e)))
    md5sum = genmsg.compute_md5(msg_context, spec)

    # remap spec names to be Python-safe
    spec = make_ruby_safe(spec)
    spec_names = spec.names

    # for not capital class like tfMessages
    name = spec.short_name
    capitalized_name = name[0].upper() + name[1:]

    # #1807 : this will be much cleaner when msggenerator library is
    # rewritten to not use globals
    clear_patterns()

    yield '# autogenerated by genrb from %s.msg. Do not edit.'%name
    yield "require 'ros/message'\n"
    import_strs = []
    for t in spec.types:
        import_strs.extend(compute_import(msg_context, spec.package, t))
    import_strs = set(import_strs)
    for i in import_strs:
        if i:
            yield i

    yield ''

    yield "module %s\n"%spec.package.capitalize()

    fulltype = '%s%s%s'%(spec.package, genmsg.SEP, name)

    #Yield data class first, e.g. Point2D
    yield 'class %s <::ROS::Message'%capitalized_name

    yield """  def self.md5sum
    \"%s\"
  end
"""%(md5sum)
    yield """  def self.type
    \"%s\"
  end
"""%(fulltype)
    if spec.has_header():
        bool_val = 'true'
    else:
        bool_val = 'false'
    yield """  def has_header?
    %s
  end
"""%bool_val
    # note: we introduce an extra newline to protect the escaping from quotes in the message
    yield """  def message_definition
    \"%s\n\"
  end"""%compute_full_text_escaped(msg_context, spec)

    if spec.constants:
        yield '  # Pseudo-constants'
        for c in spec.constants:
            if c.type == 'string':
                val = c.val
                if '"' in val and "'" in val:
                    # crude escaping of \ and "
                    escaped = c.val.replace('\\', '\\\\')
                    escaped = escaped.replace('\"', '\\"')
                    yield '  %s = "%s"'%(c.name, escaped)
                elif '"' in val: #use raw encoding for prettiness
                    yield "  %s = r'%s'"%(c.name, val)
                elif "'" in val: #use raw encoding for prettiness
                    yield '  %s = r"%s"'%(c.name, val)
                else:
                    yield "  %s = '%s'"%(c.name, val)
            else:
                yield '  %s = %s'%(c.name, c.val)
        yield ''
    yield "  attr_accessor "+", ".join([":"+x for x in spec_names])+"\n"

    yield '_REPLACE_FOR_STRUCT_'
    if len(spec_names):
        yield "  @@struct_L = ::ROS::Struct.new(\"L\")"
        yield "  @@slot_types = ['"+"','".join(spec.types)+"']"
    else:
        yield "  @@struct_L = ::ROS::Struct.new(\"L\")"
        yield "  @@slot_types = []"

    yield """
  # Constructor. You can set the default values using keyword operators.
  #
  # @param [Hash] args keyword for initializing values"""
    for (t, s) in zip(spec.types, spec_names):
        yield "  # @option args [%s] :%s initialize value"%(t, s)
    yield "  def initialize(args={})"
    if len(spec_names):
        yield "    # message fields cannot be None, assign default values for those that are"
    if len(spec_names) > 0:
      for (t, s) in zip(spec.types, spec_names):
        yield """    if args[:%s]
      @%s = args[:%s]
    else"""%(s, s, s)
        yield "      @%s = %s"%(s, default_value(msg_context, t, spec.package))
        yield "    end"
    yield "  end" # end of initialize

    yield """
  # internal API method
  # @return [String] Message type string.
  def _get_types
    @slot_types
  end

  # serialize message into buffer
  # @param [IO] buff buffer
  def serialize(buff)"""
    for y in serialize_fn_generator(msg_context, spec):
        yield "    "+ y
    yield "  end"
    yield """
  #  unpack serialized message in str into this message instance
  #  @param [String] str: byte array of serialized message
  def deserialize(str)
"""
    for y in deserialize_fn_generator(msg_context, spec):
        yield "    " + y
    yield "  end"
    yield "end # end of class"
    yield "end # end of module"

import copy
import re

def convert_to_ruby_pattern(python_pattern):
    ruby_pattern = copy.copy(python_pattern)
    # get python key
    for (py, ru) in PYTHON_RUBY_DICT.iteritems():
        ruby_pattern = re.sub(py, ru, ruby_pattern)
    # swap number and char
    return re.sub(r"([0-9]+)([A-Za-z])", r'\2\1', ruby_pattern)

def msg_generator(msg_context, spec, search_path):
    generated = ''
    for l in msg_generator_internal(msg_context, spec, search_path):
        generated += l + "\n"

    patterns = get_patterns()
    structs = ''
    for p in set(patterns):
        if p == 'I':
            continue
        ruby_p = convert_to_ruby_pattern(p)
        var_name = '  @@struct_%s'%ruby_p
        structs += '%s = ::ROS::Struct.new("%s")\n'%(var_name, ruby_p)
    clear_patterns()
    import re
    return re.sub('_REPLACE_FOR_STRUCT_', structs, generated, 1)


def srv_generator(msg_context, spec, search_path):
    for mspec in (spec.request, spec.response):
        for l in msg_generator(msg_context, mspec, search_path):
            yield l

    name = spec.short_name
    req, resp = ["%s%s"%(name, suff) for suff in ['Request', 'Response']]

    fulltype = spec.full_name

    genmsg.msg_loader.load_depends(msg_context, spec, search_path)
    md5 = genmsg.compute_md5(msg_context, spec)
    yield "module %s\n"%spec.package.capitalize()
    yield "class %s\n"%name
    yield """  def self.type
    '%s'
  end
"""%fulltype
    yield """  def self.md5sum
    '%s'
  end
"""%md5
    yield """  def self.request_class
    %s
  end
"""%req
    yield """  def self.response_class
    %s
  end
end
end"""%resp


def compute_resource_name(filename, ext):
    """
    Convert resource filename to ROS resource name
    :param filename str: path to .msg/.srv file
    :returns str: name of ROS resource
    """
    return os.path.basename(filename)[:-len(ext)]


def compute_outfile_name(outdir, infile_name, ext):
    """
    :param outdir str: path to directory that files are generated to
    :returns str: output file path based on input file name and output directory
    """
    # Use leading _ so that module name does not collide with message name. It also
    # makes it more clear that the .py file should not be imported directly
    return os.path.join(outdir, compute_resource_name(infile_name, ext)+".rb")


class Generator(object):

    def __init__(self, what, ext, spec_loader_fn, generator_fn):
        self.what = what
        self.ext = ext
        self.spec_loader_fn = spec_loader_fn
        self.generator_fn = generator_fn

    def generate(self, msg_context, full_type, f, outdir, search_path):
        try:
            # you can't just check first... race condition
            os.makedirs(outdir)
        except OSError as e:
            if e.errno != 17: # file exists
                raise
        # generate message files for request/response
        spec = self.spec_loader_fn(msg_context, f, full_type)
        outfile = compute_outfile_name(outdir, os.path.basename(f), self.ext)
        with open(outfile, 'w') as f:
            for l in self.generator_fn(msg_context, spec, search_path):
                f.write(l)
        return outfile

    def generate_messages(self, package, package_files, outdir, search_path):
        """
        :returns: return code, ``int``
        """
        if not genmsg.is_legal_resource_base_name(package):
            raise MsgGenerationException("\nERROR: package name '%s' is illegal and cannot be used in message generation.\nPlease see http://ros.org/wiki/Names"%(package))

        # package/src/package/msg for messages, packages/src/package/srv for services
        msg_context = MsgContext.create_default()
        retcode = 0
        for f in package_files:
            try:
                f = os.path.abspath(f)
                infile_name = os.path.basename(f)
                full_type = genmsg.gentools.compute_full_type_name(package, infile_name);
                outfile = self.generate(msg_context, full_type, f, outdir, search_path) #actual generation
            except Exception as e:
                if not isinstance(e, MsgGenerationException) and not isinstance(e, genmsg.msgs.InvalidMsgSpec):
                    traceback.print_exc()
                print("\nERROR: Unable to generate %s for package '%s': while processing '%s': %s\n"%(self.what, package, f, e), file=sys.stderr)
                retcode = 1 #flag error
        return retcode

class SrvGenerator(Generator):
    def __init__(self):
        super(SrvGenerator, self).__init__('services', genmsg.EXT_SRV,
                                           genmsg.msg_loader.load_srv_from_file,
                                           srv_generator)

class MsgGenerator(Generator):
    """
    Generates Python message code for all messages in a
    package. See genutil.Generator. In order to generator code for a
    single .msg file, see msg_generator.
    """
    def __init__(self):
        super(MsgGenerator, self).__init__('messages', genmsg.EXT_MSG,
                                           genmsg.msg_loader.load_msg_from_file,
                                           msg_generator)
