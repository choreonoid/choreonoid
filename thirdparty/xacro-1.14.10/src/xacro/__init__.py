# Copyright (c) 2015, Open Source Robotics Foundation, Inc.
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Open Source Robotics Foundation, Inc.
#       nor the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior
#       written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Authors: Stuart Glaser, William Woodall, Robert Haschke
# Maintainer: Morgan Quigley <morgan@osrfoundation.org>

from __future__ import print_function, division

import ast
import glob
import math
import os
import re
import sys
import xml.dom.minidom

from copy import deepcopy
from .cli import process_args
from .color import error, message, warning
from .xmlutils import opt_attrs, reqd_attrs, first_child_element, \
    next_sibling_element, replace_node


try:  # python 2
    _basestr = basestring
    encoding = {'encoding': 'utf-8'}
except NameError:  # python 3
    _basestr = str
    unicode = str
    encoding = {}

# Dictionary of substitution args
substitution_args_context = {}


# Stack of currently processed files / macros
filestack = None
macrostack = None


def init_stacks(file):
    global filestack
    global macrostack
    filestack = [file]
    macrostack = []


def abs_filename_spec(filename_spec):
    """
    Prepend the dirname of the currently processed file
    if filename_spec is not yet absolute
    """
    if not os.path.isabs(filename_spec):
        parent_filename = filestack[-1]
        basedir = os.path.dirname(parent_filename) if parent_filename else '.'
        return os.path.join(basedir, filename_spec)
    return filename_spec


class YamlListWrapper(list):
    """Wrapper class for yaml lists to allow recursive inheritance of wrapper property"""
    @staticmethod
    def wrap(item):
        """This static method, used by both YamlListWrapper and YamlDictWrapper,
           dispatches to the correct wrapper class depending on the type of yaml item"""
        if isinstance(item, dict):
            return YamlDictWrapper(item)
        elif isinstance(item, list):
            return YamlListWrapper(item)
        else:  # scalar
            return item

    def __getitem__(self, idx):
        return YamlListWrapper.wrap(super(YamlListWrapper, self).__getitem__(idx))


class YamlDictWrapper(dict):
    """Wrapper class providing dotted access to dict items"""
    def __getattr__(self, item):
        try:
            return YamlListWrapper.wrap(super(YamlDictWrapper, self).__getitem__(item))
        except KeyError:
            raise XacroException("No such key: '{}'".format(item))

    __getitem__ = __getattr__


def construct_angle_radians(loader, node):
    """utility function to construct radian values from yaml"""
    value = loader.construct_scalar(node)
    try:
        return float(safe_eval(value, _global_symbols))
    except SyntaxError:
        raise XacroException("invalid expression: %s" % value)


def construct_angle_degrees(loader, node):
    """utility function for converting degrees into radians from yaml"""
    return math.radians(construct_angle_radians(loader, node))


def load_yaml(filename):
    try:
        import yaml
        yaml.SafeLoader.add_constructor(u'!radians', construct_angle_radians)
        yaml.SafeLoader.add_constructor(u'!degrees', construct_angle_degrees)
    except Exception:
        raise XacroException("yaml support not available; install python-yaml")

    filename = abs_filename_spec(filename)
    f = open(filename)
    filestack.append(filename)
    try:
        return YamlListWrapper.wrap(yaml.safe_load(f))
    finally:
        f.close()
        filestack.pop()
        global all_includes
        all_includes.append(filename)


def tokenize(s, sep=',; ', skip_empty=True):
    results = re.split('[{}]'.format(sep), s)
    if skip_empty:
        return [item for item in results if item]
    else:
        return results


# create global symbols dictionary
# taking simple security measures to forbid access to __builtins__
# only the very few symbols explicitly listed are allowed
# for discussion, see: http://nedbatchelder.com/blog/201206/eval_really_is_dangerous.html
def create_global_symbols():
    result = dict()

    def deprecate(f, msg):
        def wrapper(*args, **kwargs):
            warning(msg)
            return f(*args, **kwargs)

        return wrapper if msg else f

    def expose(*args, **kwargs):
        # Extract args from kwargs
        source, ns, deprecate_msg = (kwargs.pop(key, None) for key in ['source', 'ns', 'deprecate_msg'])

        addons = dict()
        if source is not None:
            addons.update([(key, source[key]) for key in args])  # Add list of symbol names from source
        else:
            addons.update(*args)  # Add from list of (key, value) pairs
        addons.update(**kwargs)  # Add key=value arguments

        if ns is not None:  # Wrap dict into a namespace
            try:  # Retrieve namespace target dict
                target = result[ns]
            except KeyError:  # or create if not existing yet
                target = MacroNameSpace()
                result.update([(ns, target)])
            target.update(addons)  # Populate target dict

            if deprecate_msg is not None:  # Also import directly, but with deprecation warning
                result.update([(key, deprecate(f, deprecate_msg.format(name=key, ns=ns))) for key, f in addons.items()])
        else:
            result.update(addons)  # Import directly

    deprecate_msg = 'Using {name}() directly is deprecated. Use {ns}.{name}() instead.'
    # This is the list of symbols we have exposed for years now. Continue exposing them directly
    expose('list', 'dict', 'map', 'len', 'str', 'float', 'int', 'True', 'False', 'min', 'max', 'round',
           source=__builtins__)
    # These few were only recently added. The should move into python namespace, but (with a deprecation msg) stay global for now
    expose('sorted', 'range', source=__builtins__, ns='python', deprecate_msg=deprecate_msg)
    # Expose all builtin symbols into the python namespace. Thus the stay accessible if the global symbol was overriden
    expose('list', 'dict', 'map', 'len', 'str', 'float', 'int', 'True', 'False', 'min', 'max', 'round',
           'all', 'any', 'complex', 'divmod', 'enumerate', 'filter', 'frozenset', 'hash', 'isinstance', 'issubclass',
           'ord', 'repr', 'reversed', 'slice', 'set', 'sum', 'tuple', 'type', 'zip', source=__builtins__, ns='python')

    # Expose all math symbols and functions into namespace math (and directly for backwards compatibility -- w/o deprecation)
    expose([(k, v) for k, v in math.__dict__.items() if not k.startswith('_')], ns='math', deprecate_msg='')

    # Expose load_yaml, abs_filename, and dotify into namespace xacro (and directly with deprecation)
    expose(load_yaml=load_yaml, abs_filename=abs_filename_spec, dotify=YamlDictWrapper,
           ns='xacro', deprecate_msg=deprecate_msg)

    def message_adapter(f):
        def wrapper(*args, **kwargs):
            location = kwargs.pop('print_location', f.__name__ in ['warning', 'error'])
            kwargs.pop('file', None)  # Don't forward a file argument
            f(*args, **kwargs)
            if location:
                print_location()
            return ''  # Return empty string instead of None
        return wrapper

    def fatal(*args):
        raise XacroException(' '.join(map(str, args)))

    # Expose xacro's message functions
    expose([(f.__name__, message_adapter(f)) for f in [message, warning, error, print_location]], ns='xacro')
    expose(fatal=fatal, tokenize=tokenize, ns='xacro')

    return result


def safe_eval(expr, globals, locals=None):
    code = compile(expr.strip(), "<expression>", "eval")
    invalid_names = [n for n in code.co_names if n.startswith("__")]
    if invalid_names:
        raise XacroException("Use of invalid name(s): ", ', '.join(invalid_names))
    globals.update(__builtins__= {})  # disable default builtins
    return eval(code, globals, locals)


class XacroException(Exception):
    """
    XacroException allows to wrap another exception (exc) and to augment
    its error message: prefixing with msg and suffixing with suffix.
    str(e) finally prints: msg str(exc) suffix
    """

    def __init__(self, msg=None, suffix=None, exc=None, macro=None):
        super(XacroException, self).__init__(msg)
        self.suffix = suffix
        self.exc = exc
        self.macros = [] if macro is None else [macro]

    def __str__(self):
        items = [super(XacroException, self).__str__(), self.exc, self.suffix]
        return ' '.join([s for s in [unicode(e) for e in items] if s not in ['', 'None']])


verbosity = 1


def check_attrs(tag, required, optional):
    """
    Helper routine to fetch required and optional attributes
    and complain about any additional attributes.
    :param tag (xml.dom.Element): DOM element node
    :param required [str]: list of required attributes
    :param optional [str]: list of optional attributes
    """
    result = reqd_attrs(tag, required)
    result.extend(opt_attrs(tag, optional))
    allowed = required + optional
    extra = [a for a in tag.attributes.keys() if a not in allowed and not a.startswith("xmlns:")]
    if extra:
        warning("%s: unknown attribute(s): %s" % (tag.nodeName, ', '.join(extra)))
        if verbosity > 0:
            print_location()
    return result


class Macro(object):
    def __init__(self):
        self.body = None  # original xml.dom.Node
        self.params = []  # parsed parameter names
        self.defaultmap = {}  # default parameter values
        self.history = []  # definition history


def eval_extension(s):
    if s == '$(cwd)':
        return os.getcwd()
    try:
        from roslaunch.substitution_args import resolve_args, ArgException
        from rospkg.common import ResourceNotFound
        return resolve_args(s, context=substitution_args_context, resolve_anon=False)
    except ImportError as e:
        raise XacroException("substitution args not supported: ", exc=e)
    except ArgException as e:
        raise XacroException("Undefined substitution argument", exc=e)
    except ResourceNotFound as e:
        raise XacroException("resource not found:", exc=e)


class Table(dict):
    def __init__(self, parent=None):
        dict.__init__(self)
        if parent is None:
            parent = dict()  # Use empty dict to simplify lookup
        self.parent = parent
        try:
            self.root = parent.root  # short link to root dict / global_symbols
            self.depth = self.parent.depth + 1  # for debugging only
        except AttributeError:
            self.root = parent
            self.depth = 0
        self.unevaluated = set()  # set of unevaluated variables
        self.recursive = []  # list of currently resolved vars (to resolve recursive definitions)

    @staticmethod
    def _eval_literal(value):
        if isinstance(value, _basestr):
            # remove single quotes from escaped string
            if len(value) >= 2 and value[0] == "'" and value[-1] == "'":
                return value[1:-1]
            # Try to evaluate as number literal or boolean.
            # This is needed to handle numbers in property definitions as numbers, not strings.
            # python3 ignores/drops underscores in number literals (due to PEP515).
            # Here, we want to handle literals with underscores as plain strings.
            if '_' in value:
                return value
            for f in [int, float, lambda x: get_boolean_value(x, None)]:  # order of types is important!
                try:
                    return f(value)
                except Exception:
                    pass
        return value

    def _resolve_(self, key):
        # lazy evaluation
        if key in self.unevaluated:
            if key in self.recursive:
                raise XacroException('circular variable definition: {}\n'
                                     'Consider disabling lazy evaluation via lazy_eval="false"'
                                     .format(" -> ".join(self.recursive + [key])))
            self.recursive.append(key)
            dict.__setitem__(self, key, self._eval_literal(eval_text(dict.__getitem__(self, key), self)))
            self.unevaluated.remove(key)
            self.recursive.remove(key)

        # return evaluated result
        value = dict.__getitem__(self, key)
        if (verbosity > 2 and self.parent is self.root) or verbosity > 3:
            print("{indent}use {key}: {value} ({loc})".format(
                indent=self.depth * ' ', key=key, value=value, loc=filestack[-1]), file=sys.stderr)
        return value

    def __getitem__(self, key):
        if dict.__contains__(self, key):
            return self._resolve_(key)
        else:
            return self.parent[key]

    def _setitem(self, key, value, unevaluated):
        if key in self.root:
            warning("redefining global symbol: %s" % key)
            print_location()

        value = self._eval_literal(value)
        dict.__setitem__(self, key, value)
        if unevaluated and isinstance(value, _basestr):
            # literal evaluation failed: re-evaluate lazily at first access
            self.unevaluated.add(key)
        elif key in self.unevaluated:
            # all other types cannot be evaluated
            self.unevaluated.remove(key)
        if (verbosity > 2 and self.parent is self.root) or verbosity > 3:
            print("{indent}set {key}: {value} ({loc})".format(
                indent=self.depth * ' ', key=key, value=value, loc=filestack[-1]), file=sys.stderr)

    def __setitem__(self, key, value):
        self._setitem(key, value, unevaluated=True)

    def __delitem__(self, key):
        # Remove all items up to root
        p = self
        while p is not self.root:
            dict.pop(p, key, None)
            p = p.parent
        if key in self.root:
            warning('Cannot remove global symbol: ' + key)

    def __contains__(self, key):
        return \
            dict.__contains__(self, key) or (key in self.parent)

    def __str__(self):
        s = dict.__str__(self)
        if self.parent is not None:
            s += "\n  parent: "
            s += str(self.parent)
        return s

    def top(self):
        p = self
        while p.parent is not p.root:
            p = p.parent
        return p


class NameSpace(object):
    # dot access (namespace.property) is forwarded to getitem()
    def __getattr__(self, item):
        try:
            return self.__getitem__(item)
        except KeyError:
            raise NameError("name '{}' is not defined".format(item))


class PropertyNameSpace(Table, NameSpace):
    def __init__(self, parent=None):
        super(PropertyNameSpace, self).__init__(parent)


class MacroNameSpace(dict, NameSpace):
    def __init__(self, *args, **kwargs):
        super(MacroNameSpace, self).__init__(*args, **kwargs)


class QuickLexer(object):
    def __init__(self, *args, **kwargs):
        if args:
            # copy attributes + variables from other instance
            other = args[0]
            self.__dict__.update(other.__dict__)
        else:
            self.res = []
            for k, v in kwargs.items():
                self.__setattr__(k, len(self.res))
                self.res.append(re.compile(v))
        self.str = ""
        self.top = None

    def lex(self, str):
        self.str = str
        self.top = None
        self.next()

    def peek(self):
        return self.top

    def next(self):
        result = self.top
        self.top = None
        if not self.str:  # empty string
            return result
        for i in range(len(self.res)):
            m = self.res[i].match(self.str)
            if m:
                self.top = (i, m.group(0))
                self.str = self.str[m.end():]
                return result
        raise XacroException('invalid expression: ' + self.str)


all_includes = []
include_no_matches_msg = """Include tag's filename spec \"{}\" matched no files."""


def get_include_files(filename_spec, symbols):
    try:
        filename_spec = abs_filename_spec(eval_text(filename_spec, symbols))
    except XacroException as e:
        if e.exc and isinstance(e.exc, NameError) and symbols is None:
            raise XacroException('variable filename is supported with in-order option only')
        else:
            raise

    if re.search('[*[?]+', filename_spec):
        # Globbing behaviour
        filenames = sorted(glob.glob(filename_spec))
        if len(filenames) == 0:
            warning(include_no_matches_msg.format(filename_spec))
    else:
        # Default behaviour
        filenames = [filename_spec]

    for filename in filenames:
        global all_includes
        all_includes.append(filename)
        yield filename


def import_xml_namespaces(parent, attributes):
    """import all namespace declarations into parent"""
    for name, value in attributes.items():
        if name.startswith('xmlns:'):
            oldAttr = parent.getAttributeNode(name)
            if oldAttr and oldAttr.value != value:
                warning("inconsistent namespace redefinitions for {name}:"
                        "\n old: {old}\n new: {new} ({new_file})".format(
                            name=name, old=oldAttr.value, new=value,
                            new_file=filestack[-1]))
            else:
                parent.setAttribute(name, value)


def process_include(elt, macros, symbols, func):
    included = []
    filename_spec, namespace_spec, optional = check_attrs(elt, ['filename'], ['ns', 'optional'])
    if namespace_spec:
        try:
            namespace_spec = eval_text(namespace_spec, symbols)
            macros[namespace_spec] = ns_macros = MacroNameSpace()
            symbols[namespace_spec] = ns_symbols = PropertyNameSpace()
        except TypeError:
            raise XacroException('namespaces are supported with in-order option only')
    else:
        ns_macros = macros
        ns_symbols = symbols

    optional = get_boolean_value(optional, None)

    if first_child_element(elt):
        warning("Child elements of a <xacro:include> tag are ignored")
        if verbosity > 0:
            print_location()

    for filename in get_include_files(filename_spec, symbols):
        try:
            # extend filestack
            filestack.append(filename)
            include = parse(None, filename).documentElement

            # recursive call to func
            func(include, ns_macros, ns_symbols)
            included.append(include)
            import_xml_namespaces(elt.parentNode, include.attributes)

            # restore filestack
            filestack.pop()
        except XacroException as e:
            if e.exc and isinstance(e.exc, IOError) and optional is True:
                continue
            else:
                raise

    remove_previous_comments(elt)
    # replace the include tag with the nodes of the included file(s)
    replace_node(elt, by=included, content_only=True)


def is_valid_name(name):
    """
    Checks whether name is a valid property or macro identifier.
    With python-based evaluation, we need to avoid name clashes with python keywords.
    """
    # Resulting AST of simple identifier is <Module [<Expr <Name "foo">>]>
    try:
        root = ast.parse(name)

        if isinstance(root, ast.Module) and \
           len(root.body) == 1 and isinstance(root.body[0], ast.Expr) and \
           isinstance(root.body[0].value, ast.Name) and root.body[0].value.id == name:
            return True
    except SyntaxError:
        pass

    return False


default_value = '''\$\{.*?\}|\$\(.*?\)|(?:'.*?'|\".*?\"|[^\s'\"]+)+|'''
re_macro_arg = re.compile(r'^\s*([^\s:=]+?)\s*:?=\s*(\^\|?)?(' + default_value + ')(?:\s+|$)(.*)')
#                          space(   param )(   :=  )(  ^|  )(        default      )( space )(rest)


def parse_macro_arg(s):
    """
    parse the first param spec from a macro parameter string s
    accepting the following syntax: <param>[:=|=][^|]<default>
    :param s: param spec string
    :return: param, (forward, default), rest-of-string
             forward will be either param or None (depending on whether ^ was specified)
             default will be the default string or None
             If there is no default spec at all, the middle pair will be replaced by None
    """
    m = re_macro_arg.match(s)
    if m:
        # there is a default value specified for param
        param, forward, default, rest = m.groups()
        if not default:
            default = None
        return param, (param if forward else None, default), rest
    else:
        # there is no default specified at all
        result = s.lstrip().split(None, 1)
        return result[0], None, result[1] if len(result) > 1 else ''


def grab_macro(elt, macros):
    assert(elt.tagName == 'xacro:macro')
    remove_previous_comments(elt)

    name, params = check_attrs(elt, ['name'], ['params'])
    if name == 'call':
        raise XacroException("Invalid use of macro name 'call'")
    if name.find('.') != -1:
        raise XacroException("macro names must not contain '.' (reserved for namespaces): %s" % name)
    if name.startswith('xacro:'):
        warning("macro names must not contain prefix 'xacro:': %s" % name)
        name = name[6:]  # drop 'xacro:' prefix

    # fetch existing or create new macro definition
    macro = macros.get(name, Macro())
    # append current filestack to history
    macro.history.append(deepcopy(filestack))
    macro.body = elt

    # parse params and their defaults
    macro.params = []
    macro.defaultmap = {}
    while params:
        param, value, params = parse_macro_arg(params)
        macro.params.append(param)
        if value is not None:
            macro.defaultmap[param] = value  # parameter with default

    macros[name] = macro
    replace_node(elt, by=None)


def grab_property(elt, table):
    assert(elt.tagName == 'xacro:property')
    remove_previous_comments(elt)

    name, value, default, remove, scope, lazy_eval = \
        check_attrs(elt, ['name'], ['value', 'default', 'remove', 'scope', 'lazy_eval'])
    name = eval_text(name, table)  # Allow name to be evaluated from expression
    if not is_valid_name(name):
        raise XacroException('Property names must be valid python identifiers: ' + name)
    if name.startswith('__'):
        raise XacroException('Property names must not start with double underscore:' + name)
    remove = get_boolean_value(eval_text(remove or 'false', table), remove)
    if sum([value is not None, default is not None, remove]) > 1:
        raise XacroException('Property attributes default, value, and remove are mutually exclusive: ' + name)

    if remove and name in table:
        del table[name]
        replace_node(elt, by=None)
        return

    if default is not None:
        if scope is not None:
            warning("%s: default property value can only be defined on local scope" % name)
        if name not in table:
            value = default
        else:
            replace_node(elt, by=None)
            return

    if value is None:
        name = '**' + name
        value = elt  # debug

    replace_node(elt, by=None)

    # We use lazy evaluation by default
    lazy_eval = get_boolean_value(eval_text(lazy_eval or 'true', table), lazy_eval)

    if scope and scope == 'global':
        target_table = table.top()
        lazy_eval = False
    elif scope and scope == 'parent':
        if table.parent is not None:
            target_table = table.parent
            lazy_eval = False
        else:
            warning("%s: no parent scope at global scope " % name)
            return  # cannot store the value, no reason to evaluate it
    else:
        target_table = table

    if not lazy_eval and isinstance(value, _basestr):
        value = eval_text(value, table)  # greedily eval value

    target_table._setitem(name, value, unevaluated=lazy_eval)


LEXER = QuickLexer(DOLLAR_DOLLAR_BRACE=r"^\$\$+(\{|\()",  # multiple $ in a row, followed by { or (
                   EXPR=r"^\$\{[^\}]*\}",        # stuff starting with ${
                   EXTENSION=r"^\$\([^\)]*\)",   # stuff starting with $(
                   TEXT=r"[^$]+|\$[^{($]+|\$$")  # any text w/o $  or  $ following any chars except {($  or  single $


# evaluate text and return typed value
def eval_text(text, symbols):
    def handle_expr(s):
        try:
            return safe_eval(eval_text(s, symbols), symbols)
        except Exception as e:
            # re-raise as XacroException to add more context
            raise XacroException(exc=e,
                                 suffix=os.linesep + "when evaluating expression '%s'" % s)

    def handle_extension(s):
        return eval_extension("$(%s)" % eval_text(s, symbols))

    results = []
    lex = QuickLexer(LEXER)
    lex.lex(text)
    while lex.peek():
        id = lex.peek()[0]
        if id == lex.EXPR:
            results.append(handle_expr(lex.next()[1][2:-1]))
        elif id == lex.EXTENSION:
            results.append(handle_extension(lex.next()[1][2:-1]))
        elif id == lex.TEXT:
            results.append(lex.next()[1])
        elif id == lex.DOLLAR_DOLLAR_BRACE:
            results.append(lex.next()[1][1:])
    # return single element as is, i.e. typed
    if len(results) == 1:
        return results[0]
    # otherwise join elements to a string
    else:
        return ''.join(map(unicode, results))


def eval_default_arg(forward_variable, default, symbols, macro):
    if forward_variable is None:
        return eval_text(default, symbols)
    try:
        return symbols[forward_variable]
    except KeyError:
        if default is not None:
            return eval_text(default, symbols)
        else:
            raise XacroException("Undefined property to forward: " + forward_variable, macro=macro)


def handle_dynamic_macro_call(node, macros, symbols):
    name, = reqd_attrs(node, ['macro'])
    if not name:
        raise XacroException("xacro:call is missing the 'macro' attribute")
    name = unicode(eval_text(name, symbols))

    # remove 'macro' attribute and rename tag with resolved macro name
    node.removeAttribute('macro')
    node.tagName = 'xacro:' + name
    # forward to handle_macro_call
    handle_macro_call(node, macros, symbols)
    return True


def resolve_macro(fullname, macros):
    # split name into namespaces and real name
    namespaces = fullname.split('.')
    name = namespaces.pop(-1)

    def _resolve(namespaces, name, macros):
        # traverse namespaces to actual macros dict
        for ns in namespaces:
            macros = macros[ns]
        return macros[name]

    # try fullname and (namespaces, name) in this order
    try:
        return _resolve([], fullname, macros)
    except KeyError:
        if namespaces:
            return _resolve(namespaces, name, macros)
        else:
            raise


def handle_macro_call(node, macros, symbols):
    if node.tagName == 'xacro:call':
        return handle_dynamic_macro_call(node, macros, symbols)
    elif not node.tagName.startswith('xacro:'):
        return False  # no macro

    name = node.tagName[6:]  # drop 'xacro:' prefix
    try:
        m = resolve_macro(name, macros)
        body = m.body.cloneNode(deep=True)

    except KeyError:
        raise XacroException("unknown macro name: %s" % node.tagName)

    macrostack.append(m)

    # Expand the macro
    scoped_symbols = Table(symbols)  # new local name space for macro evaluation
    scoped_macros = Table(macros)
    params = m.params[:]  # deep copy macro's params list
    for name, value in node.attributes.items():
        if name not in params:
            raise XacroException("Invalid parameter \"%s\"" % unicode(name), macro=m)
        params.remove(name)
        scoped_symbols._setitem(name, eval_text(value, symbols), unevaluated=False)
        node.setAttribute(name, "")  # suppress second evaluation in eval_all()

    # Evaluate block parameters in node
    eval_all(node, macros, symbols)

    # Fetch block parameters, in order
    block = first_child_element(node)
    for param in params[:]:
        if param[0] == '*':
            if not block:
                raise XacroException("Not enough blocks", macro=m)
            params.remove(param)
            scoped_symbols[param] = block
            block = next_sibling_element(block)

    if block is not None:
        raise XacroException("Unused block \"%s\"" % block.tagName, macro=m)

    # Try to load defaults for any remaining non-block parameters
    for param in params[:]:
        # block parameters are not supported for defaults
        if param[0] == '*':
            continue

        # get default
        name, default = m.defaultmap.get(param, (None, None))
        if name is not None or default is not None:
            scoped_symbols._setitem(param, eval_default_arg(name, default, symbols, m), unevaluated=False)
            params.remove(param)

    if params:
        raise XacroException("Undefined parameters [%s]" % ",".join(params), macro=m)

    eval_all(body, macros, scoped_symbols)

    # Remove any comments directly before the macro call
    remove_previous_comments(node)
    # Lift all namespace attributes from the expanded body node to node's parent
    import_xml_namespaces(node.parentNode, body.attributes)
    # Replaces the macro node with the expansion
    replace_node(node, by=body, content_only=True)

    macrostack.pop()
    return True


def get_boolean_value(value, condition):
    """
    Return a boolean value that corresponds to the given Xacro condition value.
    Values "true", "1" and "1.0" are supposed to be True.
    Values "false", "0" and "0.0" are supposed to be False.
    All other values raise an exception.

    :param value: The value to be evaluated. The value has to already be evaluated by Xacro.
    :param condition: The original condition text in the XML.
    :return: The corresponding boolean value, or a Python expression that, converted to boolean, corresponds to it.
    :raises ValueError: If the condition value is incorrect.
    """
    try:
        if isinstance(value, _basestr):
            if value == 'true' or value == 'True':
                return True
            elif value == 'false' or value == 'False':
                return False
            else:
                return bool(int(value))
        else:
            return bool(value)
    except Exception:
        raise XacroException("Xacro conditional \"%s\" evaluated to \"%s\", "
                             "which is not a boolean expression." % (condition, value))


_empty_text_node = xml.dom.minidom.getDOMImplementation().createDocument(None, "dummy", None).createTextNode('\n\n')


def remove_previous_comments(node):
    """remove consecutive comments in front of the xacro-specific node"""
    next = node.nextSibling
    previous = node.previousSibling
    while previous:
        if previous.nodeType == xml.dom.Node.TEXT_NODE and \
                previous.data.isspace() and previous.data.count('\n') <= 1:
            previous = previous.previousSibling  # skip a single empty text node (max 1 newline)

        if previous and previous.nodeType == xml.dom.Node.COMMENT_NODE:
            comment = previous
            previous = previous.previousSibling
            node.parentNode.removeChild(comment)
        else:
            # insert empty text node to stop removing of comments in future calls
            # actually this moves the singleton instance to the new location
            if next and _empty_text_node != next:
                node.parentNode.insertBefore(_empty_text_node, next)
            return


def eval_all(node, macros, symbols):
    """Recursively evaluate node, expanding macros, replacing properties, and evaluating expressions"""
    # evaluate the attributes
    for name, value in node.attributes.items():
        if name.startswith('xacro:'):  # remove xacro:* attributes
            node.removeAttribute(name)
        else:
            result = unicode(eval_text(value, symbols))
            node.setAttribute(name, result)

    # remove xacro namespace definition
    try:
        node.removeAttribute('xmlns:xacro')
    except xml.dom.NotFoundErr:
        pass

    node = node.firstChild
    while node:
        next = node.nextSibling
        if node.nodeType == xml.dom.Node.ELEMENT_NODE:
            if node.tagName == 'xacro:insert_block':
                name, = check_attrs(node, ['name'], [])

                if ("**" + name) in symbols:
                    # Multi-block
                    block = symbols['**' + name]
                    content_only = True
                elif ("*" + name) in symbols:
                    # Single block
                    block = symbols['*' + name]
                    content_only = False
                else:
                    raise XacroException("Undefined block \"%s\"" % name)

                # cloning block allows to insert the same block multiple times
                block = block.cloneNode(deep=True)
                # recursively evaluate block
                eval_all(block, macros, symbols)
                replace_node(node, by=block, content_only=content_only)

            elif node.tagName == 'xacro:include':
                process_include(node, macros, symbols, eval_all)

            elif node.tagName == 'xacro:property':
                grab_property(node, symbols)

            elif node.tagName == 'xacro:macro':
                grab_macro(node, macros)

            elif node.tagName == 'xacro:arg':
                name, default = check_attrs(node, ['name', 'default'], [])
                if name not in substitution_args_context['arg']:
                    substitution_args_context['arg'][name] = eval_text(default, symbols)

                remove_previous_comments(node)
                replace_node(node, by=None)

            elif node.tagName == 'xacro:element':
                name = eval_text(*reqd_attrs(node, ['xacro:name']), symbols=symbols)
                if not name:
                    raise XacroException("xacro:element: empty name")

                node.removeAttribute('xacro:name')
                node.nodeName = node.tagName = name
                continue  # re-process the node with new tagName

            elif node.tagName == 'xacro:attribute':
                name, value = [eval_text(a, symbols) for a in reqd_attrs(node, ['name', 'value'])]
                if not name:
                    raise XacroException("xacro:attribute: empty name")

                node.parentNode.setAttribute(name, value)
                replace_node(node, by=None)

            elif node.tagName in ['xacro:if', 'xacro:unless']:
                remove_previous_comments(node)
                cond, = check_attrs(node, ['value'], [])
                keep = get_boolean_value(eval_text(cond, symbols), cond)
                if node.tagName in ['unless', 'xacro:unless']:
                    keep = not keep

                if keep:
                    eval_all(node, macros, symbols)
                    replace_node(node, by=node, content_only=True)
                else:
                    replace_node(node, by=None)

            elif handle_macro_call(node, macros, symbols):
                pass  # handle_macro_call does all the work of expanding the macro

            else:
                eval_all(node, macros, symbols)

        # TODO: Also evaluate content of COMMENT_NODEs?
        elif node.nodeType == xml.dom.Node.TEXT_NODE:
            node.data = unicode(eval_text(node.data, symbols))

        node = next


def parse(inp, filename=None):
    """
    Parse input or filename into a DOM tree.
    If inp is None, open filename and load from there.
    Otherwise, parse inp, either as string or file object.
    If inp is already a DOM tree, this function is a noop.
    :return:xml.dom.minidom.Document
    :raise: xml.parsers.expat.ExpatError
    """
    f = None
    if inp is None:
        try:
            inp = f = open(filename)
        except IOError as e:
            # do not report currently processed file as "in file ..."
            filestack.pop()
            raise XacroException(e.strerror + ": " + e.filename, exc=e)

    try:
        if isinstance(inp, _basestr):
            return xml.dom.minidom.parseString(inp)
        elif hasattr(inp, 'read'):
            return xml.dom.minidom.parse(inp)
        return inp

    finally:
        if f:
            f.close()


def process_doc(doc, mappings=None, **kwargs):
    global verbosity
    verbosity = kwargs.get('verbosity', verbosity)

    # set substitution args
    substitution_args_context['arg'] = {} if mappings is None else mappings

    # if not yet defined: initialize filestack
    if not filestack:
        init_stacks(None)

    macros = Table()
    symbols = Table(_global_symbols)

    # apply xacro:targetNamespace as global xmlns (if defined)
    targetNS = doc.documentElement.getAttribute('xacro:targetNamespace')
    if targetNS:
        doc.documentElement.removeAttribute('xacro:targetNamespace')
        doc.documentElement.setAttribute('xmlns', targetNS)

    eval_all(doc.documentElement, macros, symbols)

    # reset substitution args
    substitution_args_context['arg'] = {}


def open_output(output_filename):
    if output_filename is None:
        return sys.stdout
    else:
        dir_name = os.path.dirname(output_filename)
        if dir_name:
            try:
                os.makedirs(dir_name)
            except os.error:
                # errors occur when dir_name exists or creation failed
                # ignore error here; opening of file will fail if directory is still missing
                pass

        try:
            return open(output_filename, 'w')
        except IOError as e:
            raise XacroException("Failed to open output:", exc=e)


def print_location():
    msg = 'when instantiating macro:'
    for m in reversed(macrostack or []):
        name = m.body.getAttribute('name')
        location = '({file})'.format(file = m.history[-1][-1] or '???')
        print(msg, name, location, file=sys.stderr)
        msg = 'instantiated from:'

    msg = 'in file:' if macrostack else 'when processing file:'
    for f in reversed(filestack or []):
        if f is None:
            f = 'string'
        print(msg, f, file=sys.stderr)
        msg = 'included from:'


def process_file(input_file_name, **kwargs):
    """main processing pipeline"""
    # initialize file stack for error-reporting
    init_stacks(input_file_name)
    # parse the document into a xml.dom tree
    doc = parse(None, input_file_name)
    # perform macro replacement
    process_doc(doc, **kwargs)

    # add xacro auto-generated banner
    banner = [xml.dom.minidom.Comment(c) for c in
              [" %s " % ('=' * 83),
               " |    This document was autogenerated by xacro from %-30s | " % input_file_name,
               " |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED  %-30s | " % "",
               " %s " % ('=' * 83)]]
    first = doc.firstChild
    for comment in banner:
        doc.insertBefore(comment, first)

    return doc


_global_symbols = create_global_symbols()


def main():
    opts, input_file_name = process_args(sys.argv[1:])
    try:
        # open and process file
        doc = process_file(input_file_name, **vars(opts))
        # open the output file
        out = open_output(opts.output)

    # error handling
    except xml.parsers.expat.ExpatError as e:
        error("XML parsing error: %s" % unicode(e), alt_text=None)
        if verbosity > 0:
            print_location()
            print(file=sys.stderr)  # add empty separator line before error
            print("Check that:", file=sys.stderr)
            print(" - Your XML is well-formed", file=sys.stderr)
            print(" - You have the xacro xmlns declaration:",
                  "xmlns:xacro=\"http://www.ros.org/wiki/xacro\"", file=sys.stderr)
        sys.exit(2)  # indicate failure, but don't print stack trace on XML errors

    except Exception as e:
        msg = unicode(e)
        if not msg:
            msg = repr(e)
        error(msg)
        if verbosity > 0:
            print_location()
        if verbosity > 1:
            print(file=sys.stderr)  # add empty separator line before error
            raise  # create stack trace
        else:
            sys.exit(2)  # gracefully exit with error condition

    # special output mode
    if opts.just_deps:
        out.write(" ".join(set(all_includes)))
        print()
        return

    # write output
    out.write(doc.toprettyxml(indent='  ', **encoding))
    print()
    # only close output file, but not stdout
    if opts.output:
        out.close()
