import numpy as np
from matplotlib import pyplot as plt


def full2scale_CCS(filename_list, scale_factor):
    for i in range(0, len(filename_list)):
        file_path = "./Raw_Data/{0}.txt".format(filename_list[i])
        data = []

        with open(file_path, encoding="utf-8") as f:
            f.readline()
            f.readline()
            for line in f:
                items = [float(x) for x in line.split()]
                data.append(items)

        data = np.array(data)

        # Save full-scale
        np.savetxt(r"./Full_Scale/{0}_{1}.txt".format(filename_list[i], "FS"),
                   data,
                   fmt="%f")

        with open(r"./Full_Scale/{0}_{1}.txt".format(filename_list[i], "FS"),
                  "r+") as f:
            consent = f.read()
            f.seek(0, 0)
            f.write("[s] [m] [m/s] [m] [m/s] [rad] [rad/s] [rad/s] [rad] \n" +
                    consent)
        f.close()

        with open(r"./Full_Scale/{0}_{1}.txt".format(filename_list[i], "FS"),
                  "r+") as f:
            consent = f.read()
            f.seek(0, 0)
            f.write("time x0G u y0G v psi r delta \n" + consent)
        f.close()

        # Save Model-scale
        # time (s)
        data[:, 0] = data[:, 0] / np.sqrt(scale_factor)
        # x0G (m)
        data[:, 1] = data[:, 1] / scale_factor
        # u (m/s)
        data[:, 2] = data[:, 2] / np.sqrt(scale_factor)
        # y0G (m/s)
        data[:, 3] = data[:, 3] / scale_factor
        # v (m/s)
        data[:, 4] = data[:, 4] / np.sqrt(scale_factor)
        # psi (rad)
        data[:, 5] = data[:, 5]
        # r (rad/s)
        data[:, 6] = data[:, 6] * np.sqrt(scale_factor)
        # delta (rad)
        data[:, 7] = data[:, 7]

        scale_factor_int = int(scale_factor)

        np.savetxt(r"./Model_Scale/{0}_{1}_{2}.txt".format(
            filename_list[i], "MS", scale_factor_int),
                   data,
                   fmt="%f")

        with open(
                r"./Model_Scale/{0}_{1}_{2}.txt".format(
                    filename_list[i], "MS", scale_factor_int), "r+") as f:
            consent = f.read()
            f.seek(0, 0)
            f.write("[s] [m] [m/s] [m] [m/s] [rad] [rad/s] [rad/s] [rad] \n" +
                    consent)
        f.close()

        with open(
                r"./Model_Scale/{0}_{1}_{2}.txt".format(
                    filename_list[i], "MS", scale_factor_int), "r+") as f:
            consent = f.read()
            f.seek(0, 0)
            f.write("time x0G u y0G v psi r delta \n" + consent)
        f.close()
    return


def full2scale_SIMMAN(filename_list, scale_factor):
    for i in range(0, len(filename_list)):
        file_path = "./Raw_Data/{0}.txt".format(filename_list[i])
        data = []

        with open(file_path, encoding="utf-8") as f:
            f.readline()
            f.readline()
            for line in f:
                items = [float(x) for x in line.split()]
                data.append(items)

        data = np.array(data)

        # Save full-scale
        np.savetxt(r"./Full_Scale/{0}_{1}.txt".format(filename_list[i], "FS"),
                   data,
                   fmt="%f")

        with open(r"./Full_Scale/{0}_{1}.txt".format(filename_list[i], "FS"),
                  "r+") as f:
            consent = f.read()
            f.seek(0, 0)
            f.write("[s] [m] [m/s] [m] [m/s] [rad] [rad/s] [rad/s] [rad] \n" +
                    consent)
        f.close()

        with open(r"./Full_Scale/{0}_{1}.txt".format(filename_list[i], "FS"),
                  "r+") as f:
            consent = f.read()
            f.seek(0, 0)
            f.write("time x0G u y0G v psi r delta \n" + consent)
        f.close()

        # time (s)
        data[:, 0] = data[:, 0] / np.sqrt(scale_factor)
        # x0G (m)
        data[:, 1] = data[:, 1] / scale_factor
        # u (m/s)
        data[:, 2] = data[:, 2] / np.sqrt(scale_factor)
        # y0G (m/s)
        data[:, 3] = data[:, 3] / scale_factor
        # v (m/s)
        data[:, 4] = data[:, 4] / np.sqrt(scale_factor)
        # phi (deg)
        data[:, 5] = np.deg2rad(data[:, 5])
        # p (deg/s)
        data[:, 6] = np.deg2rad(data[:, 6]) * np.sqrt(scale_factor)
        # psi (deg)
        data[:, 7] = np.deg2rad(data[:, 7])
        # r (deg/s)
        data[:, 8] = np.deg2rad(data[:, 8]) * np.sqrt(scale_factor)
        # RPS (Hz)
        data[:, 9] = data[:, 9] * np.sqrt(scale_factor)
        # delta (deg)
        data[:, 10] = np.deg2rad(data[:, 10])

        scale_factor_int = int(scale_factor)

        np.savetxt(r"./Model_Scale/{0}_{1}_{2}.txt".format(
            filename_list[i], "MS", scale_factor_int),
                   data,
                   fmt="%f")

        with open(
                r"./Model_Scale/{0}_{1}_{2}.txt".format(
                    filename_list[i], "MS", scale_factor_int), "r+") as f:
            consent = f.read()
            f.seek(0, 0)
            f.write("[s] [m] [m/s] [m] [m/s] [rad] [rad/s] [rad/s] [rad] \n" +
                    consent)
        f.close()

        with open(
                r"./Model_Scale/{0}_{1}_{2}.txt".format(
                    filename_list[i], "MS", scale_factor_int), "r+") as f:
            consent = f.read()
            f.seek(0, 0)
            f.write("time x0G u y0G v psi r delta \n" + consent)
        f.close()
    return
