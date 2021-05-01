# -*- coding: utf-8 -*-
"""
Created on Thu Apr 29 15:05:55 2021

@author: Natha
"""
import numpy as np

from scipy.cluster.hierarchy import dendrogram, linkage
from scipy.spatial.distance import squareform

import matplotlib.pyplot as plt


mat = np.array([[0.0, 7.0, 1.0, 5.0], [7.0, 0.0, 3.0, 6.0], [1.0, 3.0, 0.0, 4.0], [5.0, 6.0, 4.0, 0.0]])
dists = squareform(mat)
linkage_matrix = linkage(dists, "complete")
dendrogram(linkage_matrix, labels=["A", "B", "C", "D"])
plt.title("test")
plt.show()