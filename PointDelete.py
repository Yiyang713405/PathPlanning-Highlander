from math import sqrt
from functools import partial
import numpy as np
import sys

if sys.version_info[0] >= 3:
    xrange = range


def pldist(point, start, end):

    if np.all(np.equal(start, end)):
        return np.linalg.norm(point - start)

    return np.divide(
        np.abs(np.linalg.norm(np.cross(end - start, start - point))),
        np.linalg.norm(end - start))


def rdp_rec(M, epsilon, dist=pldist):

    dmax = 0.0
    index = -1

    for i in xrange(1, M.shape[0]):
        d = dist(M[i], M[0], M[-1])

        if d > dmax:
            index = i
            dmax = d

    if dmax > epsilon:
        r1 = rdp_rec(M[:index + 1], epsilon, dist)
        r2 = rdp_rec(M[index:], epsilon, dist)

        return np.vstack((r1[:-1], r2))
    else:
        return np.vstack((M[0], M[-1]))


def _rdp_iter(M, start_index, last_index, epsilon, dist=pldist):
    stk = []
    stk.append([start_index, last_index])
    global_start_index = start_index
    indices = np.ones(last_index - start_index + 1, dtype=bool)

    while stk:
        start_index, last_index = stk.pop()

        dmax = 0.0
        index = start_index

        for i in xrange(index + 1, last_index):
            if indices[i - global_start_index]:
                d = dist(M[i], M[start_index], M[last_index])
                if d > dmax:
                    index = i
                    dmax = d


        if dmax > epsilon:
            stk.append([start_index, index])
            stk.append([index, last_index])
        else:
            for i in xrange(start_index + 1, last_index):
                if np.sum(indices) == 3:
                    break
                indices[i - global_start_index] = False



    return indices


def rdp_iter(M, epsilon, dist=pldist, return_mask=False):
    mask = _rdp_iter(M, 0, len(M) - 1, epsilon, dist)

    if return_mask:
        return mask

    return M[mask]


def rdp(M, epsilon=0, dist=pldist, algo="iter", return_mask=False):
    if algo == "iter":
        algo = partial(rdp_iter, return_mask=return_mask)
    elif algo == "rec":
        if return_mask:
            raise NotImplementedError("return_mask=True not supported with algo=\"rec\"")
        algo = rdp_rec

    if "numpy" in str(type(M)):
        return algo(M, epsilon, dist)

    return algo(np.array(M), epsilon, dist).tolist()


def pd(rx, ry):
    rx1 = np.array(rx)
    ry1 = np.array(ry)
    rz = np.hstack((rx1.reshape(-1, 1), ry1.reshape(-1, 1)))
    rz_rdp = rdp(rz, 1)  # Reduce the coordinate set using RDP
    rx2 = []
    ry2 = []
    for i in rz_rdp:
        rx2.append(i[0])
        ry2.append(i[1])
    return rx2, ry2


if __name__ == "__main__":
    rx = [0, 0.10000000000000009, 0.20000000000000018, 0.30000000000000004, 0.40000000000000013, 0.5, 0.6000000000000001, 0.7000000000000002]
    ry = [0, 0.08540700027163695, 0.18540700027163837, 0.2854070002716398, 0.3854070002716412, 0.4854070002716355, 0.585407000271637, 0.6854070002716384]
    rx2, ry2 = pd(rx, ry)
    print('rx2=', rx2)
    print('ry2=', ry2)
