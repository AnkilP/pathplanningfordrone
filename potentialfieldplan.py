import numpy as np
import matplotlib.pyplot as plt

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m]

def calc_potential_field(gx, gy, gz, ox, oy, oz, reso, rr):
    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    minz = min(oz) - AREA_WIDTH / 2.0
    
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    maxz = max(oz) + AREA_WIDTH / 2.0
    
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))
    zw = int(round((maxz - minz) / reso))

    # calc each potential
    pmap = [[[0.0 for i in range(yw)] for i in range(xw)] for i in range(zw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, z, gx, gy, gz):
    return 0.5 * KP * np.sqrt((x-gx)**2 + (y-gy)**2 + (z-gz)**2)


def calc_repulsive_potential(x, y, z, ox, oy, oz, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i in range(len(ox)):
        d = np.sqrt((x - ox[i])**2 + (y - oy[i])**2 + (z - oz[i]**2))
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.sqrt((x - ox[minid])**2 + (y - oy[minid])**2 + (z - oz[minid])**2)

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy, dz
    motion = [[1, 0, 0],
              [0, 1, 0],
              [-1, 0, 0],
              [0, -1, 0],
              [-1, -1, 0],
              [-1, 1, 0],
              [1, -1, 0],
              [1, 1, 0],
              [1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, 1],
              [-1, 1, 1],
              [1, -1, 1],
              [1, 1, 1],
              [1, 0, -1],
              [0, 1, -1],
              [-1, 0, -1],
              [0, -1, -1],
              [-1, -1, -1],
              [-1, 1, -1],
              [1, -1, -1],
              [1, 1, -1]]

    return motion


def potential_field_planning(sx, sy, sz, gx, gy, gz, ox, oy, oz, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, gz, ox, oy, oz, reso, rr)

    # search path
    d = np.sqrt((sx - gx)**2 + (sy - gy)**2 + (sz - gz)**2)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    # if show_animation:
        #draw_heatmap(pmap)
        # plt.plot(ix, iy, "*k")
        # plt.plot(gix, giy, "*m")

    rx, ry, rz = [sx], [sy], [sz]
    motion = get_motion_model()
    while d >= reso:
        minp = float("inf")
        minix, miniy = -1, -1
        for i in range(len(motion)):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]):
                p = float("inf")  # outside area
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

    print("Goal")

    return rx, ry, rz

# def draw_heatmap(data):
#     data = np.array(data).T
#     plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


def main():
    print("potential_field_planning start")

    sx = 0.0  # start x position [m]
    sy = 0.0  # start y positon [m]
    sz = 0.0 
    gx = 30.0  # goal x position [m]
    gy = 30.0  # goal y position [m]
    gz = 30.0
    grid_size = 0.5  # potential grid size [m]
    robot_radius = 5.0  # robot radius [m]

    ox = [15.0, 5.0, 20.0, 25.0]  # obstacle x position list [m]
    oy = [25.0, 15.0, 26.0, 25.0]  # obstacle y position list [m]
    oz = [1.0 , 2.0 , 3.0 , 4.0]

    if show_animation:
        #needs to be forwarded to vis team

    # path generation
    rx, ry, rz = potential_field_planning(
        sx, sy, sz, gx, gy, gz, ox, oy, oz, grid_size, robot_radius)

    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")