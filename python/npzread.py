import numpy as np

data = np.load('town_1_frame_2098.npz', allow_pickle=True)
lst = data.files

for item in lst:
    print(item)
    print(data[item])
