import numpy as np

class DecisionStump:

    def __init__(self):
        self.polarity = 1
        self.feature_idx = None
        self.threshhold = None
        self.alpha = None

    def predict(self, X):
        n_samples = X.shape[0]
        X_column = X[:, self.feature_idx]

        predictions = np.ones(n_samples)
        if self.polarity == 1:
            predictions[X_column < self.threshhold] =  -1
        else:
            predictions[X_column > self.threshhold] =  -1
        return predictions

class Adaboost:

    def __init__(self, n_clf=5):
        self.n_clf = n_clf

    def fit(self, X, y):
        n_samples, n_features = X.shape

        # init weights
        w = np.full(n_samples, (1/n_samples))

        self.clfs = []
        for _ in range(self.n_clf):
            clf = DecisionStump()

            min_error = float('inf')
            for feature_i in range(n_features):
                X_column = X[:, feature_i]
                thresholds = np.unique(X_column)
                for threshold in thresholds:
                    p = 1
                    predictions = np.ones(n_samples)
                    predictions[X_column < threshold] = -1

                    missclassified = w[y != predictions]
                    error = sum(missclassified)

                    if error > 0.5:
                        error = 1 - error
                        p = -1

                    if error < min_error:
                        min_error = error
                        clf.polarity = p
                        clf.threshhold = threshold
                        clf.feature_idx = feature_i

            EPS = 1e-10
            clf.alpha = 0.5 * np.log((1-min_error) / (min_error + EPS))

            predictions = clf.predict(X)

            w *= np.exp(-clf.alpha * y * predictions)
            # normalize
            w /= np.sum(w)

            self.clfs.append(clf)


    def predict(self, X):
        clf_preds = [clf.alpha * clf.predict(X) for clf in self.clfs]
        y_pred = np.sum(clf_preds, axis=0)
        y_pred = np.sign(y_pred)
        return y_pred