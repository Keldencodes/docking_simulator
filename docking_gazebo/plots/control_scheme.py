import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

sns.set(color_codes=True)

x = np.linspace(0, 0.12)

plt.figure()
plt.plot(x, 2*x)
# plt.plot(x, 2.5*x)
plt.plot(x, (5*x)**2)
plt.plot(x, (6.5*x)**3)
plt.plot(x, (7.2*x)**4)

plt.figure()
plt.plot(x, x)
plt.plot(x, (3.5*x)**2)
plt.plot(x, (5*x)**3)
plt.plot(x, (5.9*x)**4)

plt.show()
