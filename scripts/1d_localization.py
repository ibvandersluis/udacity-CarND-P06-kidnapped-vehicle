# Modified from Udacity Self-Driving Car Engineer Nanodegree
#
# Implements 1D Localisation using a finite world and rudimentary
# sensing and motion models

p = [0.2, 0.2, 0.2, 0.2, 0.2]
world = ["green", "red", "red", "green", "green"]
measurements = ["red", "red"]
motions = [1, 1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1


def sense(p, Z):
    q = []
    for i in range(len(p)):
        hit = Z == world[i]
        q.append(p[i] * (hit * pHit + (1 - hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q


def move(p, U):
    U = U % len(p)
    q = []

    for n in range(len(p)):
        q.append(
            p[n - U - 1] * pOvershoot + p[n - U] * pExact + p[n - U + 1] * pUndershoot
        )
    return q


for n in range(len(measurements)):
    p = sense(p, measurements[n])
    p = move(p, motions[n])

print(p)
