
import json
import numpy as np
import matplotlib.pyplot as plt

map = []
mwidth = 2

x0, y0 = -1, 0
for y1 in range(y0, y0+20):
    map.append([x0, y1])
y1 = y0+20
for x1 in range(x0, x0+12):
    map.append([x1, y1])
x1 = x0+12

for y2 in range(y1, y1+7):
    map.append([x1, y2])
y2 = y1+7
for x2 in range(x1, x1-22, -1):
    map.append([x2, y2])
x2 = x1-22

for y3 in range(y2, y2-15, -1):
    map.append([x2, y3])
y3 = y2-15
for x3 in range(x2, x2+5):
    map.append([x3, y3])
x3 = x2+5

for y4 in range(y3, y3-12, -1):
    map.append([x3, y4])
y4 = y3-12
for x4 in range(x3, x3+6):
    map.append([x4, y4])
x4 = x3+10

x0, y0 = 3, -6
for y1 in range(y0, y0+22):
    map.append([x0, y1])
y1 = y0+22
for x1 in range(x0, x0+14):
    map.append([x1, y1])
x1 = x0+14

for y2 in range(y1, y1+15):
    map.append([x1, y2])
y2 = y1+15
for x2 in range(x1, x1-35, -1):
    map.append([x2, y2])
x2 = x1-35

for y3 in range(y2, y2-23, -1):
    map.append([x2, y3])
y3 = y2-23
for x3 in range(x2, x2+6):
    map.append([x3, y3])
x3 = x2+6

for y4 in range(y3, y3-14, -1):
    map.append([x3, y4])
y4 = y3-14
for x4 in range(x3, x3+16):
    map.append([x4, y4])
x4 = x3+16

for x5 in range(-1, 3):
    map.append([x5, 0])

map = np.array(map)
np.save('toymap.npy', map)
# map = np.load('toymap.npy')

mapdic = {
  'map': 'toymap.npy',
  'start': (1, 2),
  'goal': (1, -2)
}
with open('toymap.json', 'w') as f:
    json.dump(mapdic, f)


def draw_map(map, start, goal, path=None, wfil='toymap.png'):
    plt.figure(figsize=(5,5))
    plt.scatter(map[:,0], map[:,1])
    plt.scatter([start[0]], [start[1]], label='start')
    plt.scatter([goal[0]], [goal[1]], label='goal')
    if path is not None:
        plt.plot(path, label='path')
    plt.legend()
    plt.savefig(wfil)

draw_map(map, mapdic['start'], mapdic['goal'])
