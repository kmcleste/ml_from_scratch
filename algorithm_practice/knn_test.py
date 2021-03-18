import numpy as np
from sklearn import datasets
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from knn import KNN

cmap = ListedColormap(['#FF0000', '#00FF00', '#0000FF'])

iris = datasets.load_iris()
X, y = iris.data, iris.target

X_train, X_test, y_train, y_test = train_test_split(X,y, test_size=0.2, random_state=1234)

# print(X_train.shape)
# print(X_train[0])

# print(y_train.shape)
# print(y_train)

# plt.figure()
# plt.scatter(X[:,0], X[:, 1], c=y, cmap=cmap, edgecolor='k', s=20) # This plots only the first 2 features hence [:,0] and [:,1]
# plt.show()

# a = [1, 1, 1, 2, 2, 4, 5, 6]
# from collections import Counter
# most_common = Counter(a).most_common(1) # the val in most_common dictates how the number of different variables being counted for most common
# print(most_common[0][0])

clf = KNN(k=7)
clf.fit(X_train, y_train)
predictions = clf.predict(X_test)

accuracy = np.sum(predictions == y_test) / len(y_test) # for each prediction that is the same as the label, it adds 1, then divides by test samples
print(accuracy)