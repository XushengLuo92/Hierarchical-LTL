import numpy as np
import matplotlib.pyplot as plt

q_profile = np.loadtxt('./analysis/q_profile.txt', delimiter=',')

ax = plt.subplot(111)
ax.plot(q_profile[:, 0])
ax.plot(q_profile[:, 1])
ax.plot(q_profile[:, 2])
ax.plot(q_profile[:, 3])
ax.plot(q_profile[:, 4])
ax.plot(q_profile[:, 5])

plt.show()