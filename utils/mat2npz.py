import scipy.io
import numpy as np
mat = scipy.io.loadmat("../data/trajectories/force_closure_1111.mat")
data: dict = {"k": [], "q0": [], "qd0": [], "qdd0": []}
for i in range(mat["k_opt"].shape[1]):
    data["q0"].append(mat["state"][:7, 50 * i])
    data["qd0"].append(mat["state"][7:14, 50 * i])
    data["qdd0"].append(mat["state"][14:, 50 * i])
    data["k"].append(mat["k_opt"][0, i].reshape(-1))

data["q0"] = np.array(data["q0"])
data["qd0"] = np.array(data["qd0"])
data["qdd0"] = np.array(data["qdd0"])
data["k"] = np.array(data["k"]) * np.pi / 72
data["q0"][:, 0] += np.pi

np.savez_compressed("../data/trajectories/force_closure_1111.npz", **data)
print("done")