import csv
import numpy as np
import pandas as pd

FILE_NAME = "spambase.data"

# Load file manually using csv built into python, usually slower than other opts
# with open(FILE_NAME, 'r') as f:
#     data = list(csv.reader(f, delimiter=","))
# data = np.array(data)

#data = np.loadtxt(FILE_NAME, delimiter=",")

data = np.genfromtxt(FILE_NAME, delimiter=",", dtype=np.float32)

print(data.shape, type(data[0][0]))
n_samples, n_features = data.shape
n_features -= 1
X = data[:,0:n_features]
y = data[:, n_features]
print(X.shape, y.shape)

df = pd.read_csv(FILE_NAME, header=None, delimiter=",", dtype=np.float32)
data = df.to_numpy()
n_samples, n_features = data.shape
n_features -= 1
X = data[:,0:n_features]
y = data[:, n_features]
print(X.shape, y.shape)
