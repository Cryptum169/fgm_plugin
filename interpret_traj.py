with open("read.txt",'r') as file:
    data = [x[:-1] for x in file.readlines()[3:]]
    # print(data)

x_val = [k.split('\t')[2] for k in data]
y_val = [k.split('\t')[3] for k in data]

x_d_val = [k.split('\t')[-2] for k in data]
y_d_val = [k.split('\t')[-1] for k in data]

import matplotlib.pyplot as plt

plt.plot(x_val, y_val)
plt.plot(x_d_val, y_d_val)
plt.show()