# This file was automatically generated by SWIG (http://www.swig.org).
# Version 4.0.0
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info

# import _rehamovelib

if _swig_python_version_info < (2, 7, 0):
    raise RuntimeError('Python 2.7 or later required')

# Import the low-level C/C++ module
if __package__ or '.' in __name__:
    from . import _rehamovelib
else:
    import _rehamovelib

try:
    import builtins as __builtin__
except ImportError:
    import builtins

def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if name == "thisown":
        return self.this.own(value)
    if name == "this":
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if not static:
        object.__setattr__(self, name, value)
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr(self, class_type, name):
    if name == "thisown":
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    raise AttributeError("'%s' object has no attribute '%s'" % (class_type.__name__, name))


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)


def _swig_setattr_nondynamic_instance_variable(set):
    def set_instance_attr(self, name, value):
        if name == "thisown":
            self.this.own(value)
        elif name == "this":
            set(self, name, value)
        elif hasattr(self, name) and isinstance(getattr(type(self), name), property):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add instance attributes to %s" % self)
    return set_instance_attr


def _swig_setattr_nondynamic_class_variable(set):
    def set_class_attr(cls, name, value):
        if hasattr(cls, name) and not isinstance(getattr(cls, name), property):
            set(cls, name, value)
        else:
            raise AttributeError("You cannot add class attributes to %s" % cls)
    return set_class_attr


def _swig_add_metaclass(metaclass):
    """Class decorator for adding a metaclass to a SWIG wrapped class - a slimmed down version of six.add_metaclass"""
    def wrapper(cls):
        return metaclass(cls.__name__, cls.__bases__, cls.__dict__.copy())
    return wrapper


class _SwigNonDynamicMeta(type):
    """Meta class to enforce nondynamic attributes (no new attributes) for a class"""
    __setattr__ = _swig_setattr_nondynamic_class_variable(type.__setattr__)


class RehamoveDevice(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    device = property(_rehamovelib.RehamoveDevice_device_get, _rehamovelib.RehamoveDevice_device_set)
    battery = property(_rehamovelib.RehamoveDevice_battery_get, _rehamovelib.RehamoveDevice_battery_set)
    mode = property(_rehamovelib.RehamoveDevice_mode_get, _rehamovelib.RehamoveDevice_mode_set)
    current = property(_rehamovelib.RehamoveDevice_current_get, _rehamovelib.RehamoveDevice_current_set)
    pulse_width = property(_rehamovelib.RehamoveDevice_pulse_width_get, _rehamovelib.RehamoveDevice_pulse_width_set)

    def __init__(self):
        _rehamovelib.RehamoveDevice_swiginit(self, _rehamovelib.new_RehamoveDevice())
    __swig_destroy__ = _rehamovelib.delete_RehamoveDevice

# Register RehamoveDevice in _rehamovelib:
_rehamovelib.RehamoveDevice_swigregister(RehamoveDevice)


def open_port(port_name):
    return _rehamovelib.open_port(port_name)

def close_port(r):
    return _rehamovelib.close_port(r)

def initialize_low_level(r):
    return _rehamovelib.initialize_low_level(r)

def stop_low_level(r):
    return _rehamovelib.stop_low_level(r)

def pulse(r, channel, current, pulse_width):
    return _rehamovelib.pulse(r, channel, current, pulse_width)

def custom_pulse(r, channel, num_points, c0, w0, c1, w1, c2, w2, c3, w3, c4, w4, c5, w5, c6, w6, c7, w7, c8, w8, c9, w9, c10, w10, c11, w11, c12, w12, c13, w13, c14, w14, c15, w15):
    return _rehamovelib.custom_pulse(r, channel, num_points, c0, w0, c1, w1, c2, w2, c3, w3, c4, w4, c5, w5, c6, w6, c7, w7, c8, w8, c9, w9, c10, w10, c11, w11, c12, w12, c13, w13, c14, w14, c15, w15)

def change_mode(r, mode):
    return _rehamovelib.change_mode(r, mode)

def set_pulse_data(r, current, pulse_width):
    return _rehamovelib.set_pulse_data(r, current, pulse_width)

def run(r, channel, period, total_milliseconds):
    return _rehamovelib.run(r, channel, period, total_milliseconds)

def midlevel_start(r, channel, period):
    return _rehamovelib.midlevel_start(r, channel, period)

def midlevel_update(r):
    return _rehamovelib.midlevel_update(r)

def midlevel_end(r):
    return _rehamovelib.midlevel_end(r)

def get_version():
    return _rehamovelib.get_version()

def get_battery(r):
    return _rehamovelib.get_battery(r)

def get_mode(r):
    return _rehamovelib.get_mode(r)

def get_current(r):
    return _rehamovelib.get_current(r)

def get_pulse_width(r):
    return _rehamovelib.get_pulse_width(r)

def battery_request(r):
    return _rehamovelib.battery_request(r)

