import numpy as np
from numpy.lib.index_tricks import nd_grid
from sklearn.datasets import make_blobs
import matplotlib.pyplot as plt
from kmeans import KMeans

X, y = make_blobs(centers=4, n_samples=500, n_features=2, shuffle=True, random_state=42)
print(X.shape)

clusters = len(np.unique(y))
print(clusters)

k = KMeans(K = clusters, max_iters=150, plot_steps=False)
y_pred = k.predict(X)

k.plot()