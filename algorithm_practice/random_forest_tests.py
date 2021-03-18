import numpy as np
from sklearn import datasets
from sklearn.model_selection import train_test_split
import time

from random_forest import RandomForest

def accuracy(y_true, y_pred):
    accuracy = np.sum(y_true == y_pred) / len(y_true)
    return accuracy

data = datasets.load_breast_cancer()
X = data.data
y = data.target

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=1234)

starttime = time.time()

clf = RandomForest(n_trees=3)
clf.fit(X_train, y_train)

y_pred = clf.predict(X_test)

endtime = time.time()
t = endtime - starttime

acc = accuracy(y_test, y_pred)

print("Accuracy: ", acc)
print("Train and fit time: ", t)