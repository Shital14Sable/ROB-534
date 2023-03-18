import matplotlib.pyplot as plt
import scipy.stats as stats

lower, upper = -2, 2
mu, sigma = 0, 2

X = stats.truncnorm(
    (lower - mu) / sigma, (upper - mu) / sigma, loc=mu, scale=sigma)
N = stats.norm(loc=mu, scale=sigma)

fig, ax = plt.subplots(2, sharex=True)
ax[0].hist(X.rvs(10000))
ax[1].hist(N.rvs(10000))
plt.show()
