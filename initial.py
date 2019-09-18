import numpy.random
keys = [
"VT_RESET_CD0",
"VT_RESET_CD1",
"VT_RESET_CD2",
"VT_RESET_CL0",
"VT_RESET_CL1",
"VT_RESET_CTDZ",
"VT_RESET_CTX",
"VT_RESET_CTZ",
]

th0 = {
"VT_RESET_CD0": 0.1543,
"VT_RESET_CD1": 0.178,
"VT_RESET_CD2": 1.619,
"VT_RESET_CL0": 0.3707,
"VT_RESET_CL1": 3.2566,
"VT_RESET_CTX": 20.588889,
"VT_RESET_CTZ": 82.3555555556,
"VT_RESET_CTDZ": 0.136,
}

s = {
"VT_RESET_CD0": 4,
"VT_RESET_CD1": 4,
"VT_RESET_CD2": 2,
"VT_RESET_CL0": 4,
"VT_RESET_CL1": 2,
"VT_RESET_CTX": 2,
"VT_RESET_CTZ": 2,
"VT_RESET_CTDZ": 4,
}


for k in keys:
    v = th0[k]
    print(k, numpy.random.uniform(v/s[k], v*s[k]))
