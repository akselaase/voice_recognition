import builtins
import logging
import sys


class Window:
    def __init__(self, maxwidth, data, sample_func=lambda x: x*x):
        self.data = data
        self.dataindex = 0
        self.maxwidth = maxwidth
        self.currwidth = min(len(data), maxwidth)
        self.sample_func = sample_func
        self.cumulative = self.calc_sum()

    def sum(self):
        return self.cumulative

    def calc_sum(self):
        return builtins.sum(map(self.sample_func, self.aslist()))

    def window_width(self):
        return min(self.currwidth, len(self.data) - self.start(), self.maxwidth)

    def is_maximized(self):
        return self.window_width() == self.maxwidth

    def start(self):
        return self.dataindex

    def end(self):
        return self.start() + self.window_width()

    def get(self, index):
        if index >= self.window_width() or -index > self.window_width():
            raise Exception("Index out of bounds")
        if index >= 0:
            return self.data[self.start() + index]
        elif index < 0:
            return self.data[self.end() + index]

    def movetostart(self):
        self.dataindex = 0
        self.currwidth = min(len(self.data), self.maxwidth)
        self.cumulative = self.calc_sum()

    def movetoend(self):
        self.dataindex = len(self.data)
        self.currwidth = 0
        self.cumulative = self.calc_sum()

    def step(self):
        if self.end() >= len(self.data):
            return
        if self.currwidth < self.maxwidth:
            # we're below max size, expand the window by one
            self.currwidth += 1
            self.cumulative += self.sample_func(self.get(-1))
        else:
            # we're at max size, move ahead by one
            prev = self.get(0)
            self.dataindex += 1
            new = self.get(-1)
            self.cumulative += self.sample_func(new) - self.sample_func(prev)

    def aslist(self):
        return self.data[self.start():self.end()]
