from scipy.stats import norm
mindis = -4.1
std = 1
a = norm.cdf(mindis, 0.0, std)
print(a)