
from numbers import Number
from enum import Enum

class ParameterType:

    def __init__(self, name, ptype):
        self._name = name
        self._cast = ptype

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._cast

class Parameter(ParameterType):
    def __init__(self, name, value, ptype=None):
        if ptype is None:
            ptype = type(value)
        super().__init__(name, ptype)
        self._value = self._cast(value)
        self._observer = []

    def add_observer(self, observer):
        self._observer.append(observer)

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        print("Set parameter '{}' = {}".format(self.name, value))
        old_value = self._value
        self._value = self._cast(value)
        # self._value = min(max(value, self.min), self.max)
        if old_value != self._value:
            for cb in self._observer:
                cb(old_value, self._value)


class RangeParameter(Parameter):

    def __init__(self, name, value, min_value, max_value, ptype=None):
        super().__init__(name, value, ptype)
        assert issubclass(self._cast, Number)
        self._range = (self._cast(min_value), self._cast(max_value))

    @property
    def min(self):
        return self._range[0]

    @property
    def max(self):
        return self._range[1]


class BoolParameter(Parameter):

    def __init__(self, name, value):
        super().__init__(name, value, bool)

    def toggle(self):
        self.value = not self.value


class OptionParameter(Parameter):

    def __init__(self, name, value, options=None, ptype=None):
        if issubclass(value, IntEnum) and options is None:
            options = list(type(select))
        super().__init__(name, value, ptype)
        assert all([issubclass(opt, self._cast) for opt in options])
        self._options = options

    @property
    def options(self):
        return list(self._options)

    def __getitem__(self, index):
        return self.options[index]

    @property
    def number(self):
        return len(self.options)



