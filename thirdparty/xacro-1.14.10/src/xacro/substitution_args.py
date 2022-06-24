# Software License Agreement (BSD License)
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
# Revision $Id: substitution_args.py 15178 2011-10-10 21:22:53Z kwc $

"""
Library for processing XML substitution args. This is currently used
by roslaunch and xacro, but it is not yet a top-level ROS feature.
"""

import os

try:
    from cStringIO import StringIO # Python 2.x
except ImportError:
    from io import StringIO # Python 3.x

import math

class SubstitutionException(Exception):
    """
    Base class for exceptions in substitution_args routines
    """
    pass

class ArgException(SubstitutionException):
    """
    Exception for missing $(arg) values
    """
    pass

# rospkg.common
class ResourceNotFound(Exception):
    """
    A ROS filesystem resource was not found.
    """
    pass

def _eval_env(name):
    try:
        return os.environ[name]
    except KeyError as e:
        raise SubstitutionException("environment variable %s is not set" % str(e))


def _env(resolved, a, args, context):
    """
    process $(env) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) != 1:
        raise SubstitutionException("$(env var) command only accepts one argument [%s]"%a)
    return resolved.replace("$(%s)" % a, _eval_env(args[0]))


def _eval_optenv(name, default=''):
    if name in os.environ:
        return os.environ[name]
    return default


def _optenv(resolved, a, args, context):
    """
    process $(optenv) arg
    @return: updated resolved argument
    @rtype: str
    @raise SubstitutionException: if arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(optenv var) must specify an environment variable [%s]"%a)
    return resolved.replace("$(%s)" % a, _eval_optenv(args[0], default=' '.join(args[1:])))


def _eval_find(pkg):
    ros_package_path = os.getenv('ROS_PACKAGE_PATH')
    if ros_package_path == "":
        raise ResourceNotFound
    
    package_paths = ros_package_path.split(':')
    for package_path in package_paths:
        if package_path.endswith('/' + pkg):
            return package_path

    raise ResourceNotFound


def _find(resolved, a, args, context):
    """
    process $(find PKG)
    Resolves the path while considering the path following the command to provide backward compatible results.
    :returns: updated resolved argument, ``str``
    :raises: :exc:SubstitutionException: if PKG invalidly specified
    :raises: :exc:`rospkg.ResourceNotFound` If PKG requires resource (e.g. package) that does not exist
    """
    if len(args) != 1:
        raise SubstitutionException("$(find pkg) command only accepts one argument [%s]" % a)

    return resolved.replace("$(%s)" % a, _eval_find(args[0]))


def _eval_arg(name, args):
    try:
        return args[name]
    except KeyError:
        raise ArgException(name)


def _arg(resolved, a, args, context):
    """
    process $(arg) arg
    
    :returns: updated resolved argument, ``str``
    :raises: :exc:`ArgException` If arg invalidly specified
    """
    if len(args) == 0:
        raise SubstitutionException("$(arg var) must specify a variable name [%s]"%(a))
    elif len(args) > 1:
        raise SubstitutionException("$(arg var) may only specify one arg [%s]"%(a))
    
    if 'arg' not in context:
        context['arg'] = {}
    return resolved.replace("$(%s)" % a, _eval_arg(name=args[0], args=context['arg']))

# Create a dictionary of global symbols that will be available in the eval
# context.  We disable all the builtins, then add back True and False, and also
# add true and false for convenience (because we accept those lower-case strings
# as boolean values in XML).
_eval_dict={
    'true': True, 'false': False,
    'True': True, 'False': False,
    '__builtins__': {k: __builtins__[k] for k in ['list', 'dict', 'map', 'str', 'float', 'int']},
    'env': _eval_env,
    'optenv': _eval_optenv,
    'find': _eval_find
}
# also define all math symbols and functions
_eval_dict.update(math.__dict__)

class _DictWrapper(object):
    def __init__(self, args, functions):
        self._args = args
        self._functions = functions

    def __getitem__(self, key):
        try:
            return self._functions[key]
        except KeyError:
            return convert_value(self._args[key], 'auto')

def _eval(s, context):
    if 'arg' not in context:
        context['arg'] = {}

    # inject arg context
    def _eval_arg_context(name): return convert_value(_eval_arg(name, args=context['arg']), 'auto')
    functions = {
        'arg': _eval_arg_context,
    }
    functions.update(_eval_dict)

    # ignore values containing double underscores (for safety)
    # http://nedbatchelder.com/blog/201206/eval_really_is_dangerous.html
    if s.find('__') >= 0:
        raise SubstitutionException("$(eval ...) may not contain double underscore expressions")
    return str(eval(s, {}, _DictWrapper(context['arg'], functions)))


# This function is copied from roslaunch.loader
# See https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/loader.py
def convert_value(value, type_):
    """
    Convert a value from a string representation into the specified
    type
    
    @param value: string representation of value
    @type  value: str
    @param type_: int, double, string, bool, or auto
    @type  type_: str
    @raise ValueError: if parameters are invalid
    """
    type_ = type_.lower()
    # currently don't support XML-RPC date, dateTime, maps, or list
    # types
    if type_ == 'auto':
        #attempt numeric conversion
        try:
            if '.' in value:
                return float(value)
            else:
                return int(value)
        except ValueError as e:
            pass
        #bool
        lval = value.lower()
        if lval == 'true' or lval == 'false':
            return convert_value(value, 'bool')
        #string
        return value
    elif type_ == 'str' or type_ == 'string':
        return value
    elif type_ == 'int':
        return int(value)
    elif type_ == 'double':
        return float(value)
    elif type_ == 'bool' or type_ == 'boolean':
        value = value.lower().strip()
        if value == 'true' or value == '1':
            return True
        elif value == 'false' or value == '0':
            return False
        raise ValueError("%s is not a '%s' type"%(value, type_))
    elif type_ == 'yaml':
        try:
            return yaml.safe_load(value)
        except yaml.parser.ParserError as e:
            raise ValueError(e)
    else:
        raise ValueError("Unknown type '%s'"%type_)  


def resolve_args(arg_str, context=None, resolve_anon=True, filename=None):
    """
    Resolves substitution args (see wiki spec U{http://ros.org/wiki/roslaunch}).

    @param arg_str: string to resolve zero or more substitution args
        in. arg_str may be None, in which case resolve_args will
        return None
    @type  arg_str: str
    @param context dict: (optional) dictionary for storing results of
        the 'anon' and 'arg' substitution args. multiple calls to
        resolve_args should use the same context so that 'anon'
        substitions resolve consistently. If no context is provided, a
        new one will be created for each call. Values for the 'arg'
        context should be stored as a dictionary in the 'arg' key.
    @type  context: dict
    @param resolve_anon bool: If True (default), will resolve $(anon
        foo). If false, will leave these args as-is.
    @type  resolve_anon: bool

    @return str: arg_str with substitution args resolved
    @rtype:  str
    @raise SubstitutionException: if there is an error resolving substitution args
    """
    if context is None:
        context = {}
    if not arg_str:
        return arg_str
    # special handling of $(eval ...)
    if arg_str.startswith('$(eval ') and arg_str.endswith(')'):
        return _eval(arg_str[7:-1], context)
    # first resolve variables like 'env' and 'arg'
    commands = {
        'env': _env,
        'optenv': _optenv,
        'arg': _arg,
    }
    resolved = _resolve_args(arg_str, context, resolve_anon, commands)
    # then resolve 'find' as it requires the subsequent path to be expanded already
    commands = {
        'find': _find,
    }
    resolved = _resolve_args(resolved, context, resolve_anon, commands)
    return resolved

def _resolve_args(arg_str, context, resolve_anon, commands):
    valid = ['find', 'env', 'optenv', 'arg']
    resolved = arg_str
    for a in _collect_args(arg_str):
        splits = [s for s in a.split(' ') if s]
        if not splits[0] in valid:
            raise SubstitutionException("Unknown substitution command [%s]. Valid commands are %s"%(a, valid))
        command = splits[0]
        args = splits[1:]
        if command in commands:
            resolved = commands[command](resolved, a, args, context)
    return resolved

_OUT  = 0
_DOLLAR = 1
_LP = 2
_IN = 3
def _collect_args(arg_str):
    """
    State-machine parser for resolve_args. Substitution args are of the form:
    $(find package_name)/scripts/foo.py $(export some/attribute blar) non-relevant stuff
    
    @param arg_str: argument string to parse args from
    @type  arg_str: str
    @raise SubstitutionException: if args are invalidly specified
    @return: list of arguments
    @rtype: [str]
    """
    buff = StringIO()
    args = []
    state = _OUT
    for c in arg_str:
        # No escapes supported
        if c == '$':
            if state == _OUT:
                state = _DOLLAR
            elif state == _DOLLAR:
                pass
            else:
                raise SubstitutionException("Dollar signs '$' cannot be inside of substitution args [%s]"%arg_str)
        elif c == '(':
            if state == _DOLLAR:
                state = _LP
            elif state != _OUT:
                raise SubstitutionException("Invalid left parenthesis '(' in substitution args [%s]"%arg_str)
        elif c == ')':
            if state == _IN:
                #save contents of collected buffer
                args.append(buff.getvalue())
                buff.truncate(0)
                buff.seek(0)
                state = _OUT
            else:
                state = _OUT
        elif state == _DOLLAR:
            # left paren must immediately follow dollar sign to enter _IN state
            state = _OUT
        elif state == _LP:
            state = _IN

        if state == _IN:
            buff.write(c)
    return args
