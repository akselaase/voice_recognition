#!/usr/bin/env python3
import pickle
import matplotlib.pyplot as plt
import numpy as np

def plot(h, l, width=1600):
    plt.plot(np.array(pickle.load(open('sums.bin', 'rb')))/32768**2/width)
    plt.axhline(l/100)
    plt.axhline(h/100)
    plt.show()

hi, lo = map(float, open('thresholds.txt').read().split(','))
plot(hi, lo)
