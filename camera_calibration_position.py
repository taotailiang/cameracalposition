# coding=utf-8
import cv2
import numpy as np
import time
import camera_configs
import time
import logging
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

params = cv2.aruco.DetectorParameters_create()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
params.cornerRefinementWinSize = 9
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

DEPTH1 = -0
DEPTH2 = -100

u1 = 771
v1 = 348
u2 = 1820
v2 = 275
# u3=1002
# v3=1998
u3 = 1957
v3 = 1960
x1 = 51.245
y1 = 251.81
x2 = 291.244
y2 = 251.81
x3 = 303.2434
y3 = -126.1871
MARK_LEN = 25

MOVELIST = [[51.245, 251.81],
            [291.244, 251.81],
            [87.22, -124.79],
            [303.24, -126.1871]]
MOVELIST = np.array(MOVELIST)
CAMERAUVLIST = [[771, 348],
                [1820, 275],
                [1002, 1998],
                [1957.0, 1960.0]]

X = [2.27993937e-01, -1.14158869e-02, -1.20565597e+02]
Y = [-1.55233551e-02, -2.23068487e-01, 3.41406340e+02]

x_1cm = (MOVELIST[1]-MOVELIST[0])/24
y_1cm = (MOVELIST[3]-MOVELIST[1])/37.5

LittleBox = np.empty([5, 7, 2], dtype=float)
RecBox = np.empty([2, 4, 2], dtype=float)
BigBox = np.empty([2, 5, 2], dtype=float)


def reflashframe():
    print("hello")


def reflashLocation():
        # cal movelist new
    MOVELISTLAST = MOVELIST
    u, v = get_erweimaUV(20)
    #u,v = 786.2,290.1
    if u != 0:
        print("20#OK")
        MOVELIST[0][0] = X[0]*(u-CAMERAUVLIST[0][0]) + \
            X[1]*(v-CAMERAUVLIST[0][1])+MOVELIST[0][0]
        MOVELIST[0][1] = Y[0]*(u-CAMERAUVLIST[0][0]) + \
            Y[1]*(v-CAMERAUVLIST[0][1])+MOVELIST[0][1]

    print(u, v)
    print(MOVELIST[0][0])
    u, v = get_erweimaUV(46)
    #u,v = 1836,287.3
    if u != 0:
        print("46#OK")
        MOVELIST[1][0] = X[0]*(u-CAMERAUVLIST[1][0]) + \
            X[1]*(v-CAMERAUVLIST[1][1])+MOVELIST[1][0]
        MOVELIST[1][1] = Y[0]*(u-CAMERAUVLIST[1][0]) + \
            Y[1]*(v-CAMERAUVLIST[1][1])+MOVELIST[1][1]
    u, v = get_erweimaUV(12)
    #u,v = 916.5,1953
    if u != 0:
        print("12#OK")
        MOVELIST[2][0] = X[0]*(u-CAMERAUVLIST[2][0]) + \
            X[1]*(v-CAMERAUVLIST[2][1])+MOVELIST[2][0]
        MOVELIST[2][1] = Y[0]*(u-CAMERAUVLIST[2][0]) + \
            Y[1]*(v-CAMERAUVLIST[2][1])+MOVELIST[2][1]
    u, v = get_erweimaUV(33)
    #u,v = 1869,1973
    if u != 0:
        print("33#OK")
        MOVELIST[3][0] = X[0]*(u-CAMERAUVLIST[3][0]) + \
            X[1]*(v-CAMERAUVLIST[3][1])+MOVELIST[3][0]
        MOVELIST[3][1] = Y[0]*(u-CAMERAUVLIST[3][0]) + \
            Y[1]*(v-CAMERAUVLIST[3][1])+MOVELIST[3][1]
    # reflash x_1cm and y_1cm
    x_1cm = (MOVELIST[1]-MOVELIST[0])/24
    y_1cm = (MOVELIST[3]-MOVELIST[1])/37.5
    # cal littlebox
    for i in range(0, 7):
        LittleBox[0][i] = (MOVELIST[1]-MOVELIST[0])*i/6 + \
            (MOVELIST[0]+x_1cm*1.5+y_1cm*2.5)
    for i in range(1, 5):
        for j in range(0, 7):
            LittleBox[i][j] = LittleBox[0][j]+y_1cm*4*i
    # cal bigbox
    BigBox[0][0] = MOVELIST[2]+x_1cm*2.5-y_1cm*3-y_1cm*5
    for i in range(1, 5):
        BigBox[0][i] = BigBox[0][0]+x_1cm*5*i
    for i in range(0, 5):
        BigBox[1][i] = BigBox[0][i]+y_1cm*5
    # cal recbox
    RecBox[0][0] = MOVELIST[2]+x_1cm*3.3-y_1cm*15.2  # -y_1cm*5
    for i in range(1, 4):
        RecBox[0][i] = RecBox[0][0]+x_1cm*6.1*i
    for i in range(0, 4):
        RecBox[1][i] = RecBox[0][i]+y_1cm*3.2


def getLittleBox(row, line):
    return LittleBox[row][line]


def getBigBox(row, line):
    return BigBox[row][line]


def getRecBox(row, line):
    return RecBox[row][line]


def getRefPosition(num):
    return MOVELIST[num]


def getAllBoxPosition(row, line, arm):
    return 0, 0


def printxy(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        # print(x-1920,y-1080)
        print(x, y)


def get_erweimaUV(NUM):

    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 9
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    cap0 = cv2.VideoCapture(0)
    cap0.set(3, 3840)
    cap0.set(4, 2160)
    cap0.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap0.set(cv2.CAP_PROP_EXPOSURE, 0)
    try:
        ret0, frame0 = cap0.read()
    except:
        print("camera open error")
        return
    img0_rectified = cv2.remap(
        frame0, camera_configs.left_map1, camera_configs.left_map2, cv2.INTER_LINEAR)
    corners, ids, rejected = cv2.aruco.detectMarkers(
        img0_rectified, dictionary, parameters=params)
    if len(corners) == 0:
        print("no corners")
        return
    kp = np.array(corners)
    ids = ids.flatten().tolist()
    try:
        kp = np.sum(kp[ids.index(NUM)], axis=1).flatten() / 4
        return kp
    except:
        print("there is no kp")
        kp = 0, 0
        return kp


def get_Real_XY(x1, y1, x2, y2):
    x3 = 2.3*(x2-x1)+x1
    y3 = 2.3*(y2-y1)+y1
    return x3, y3


def get_TCP_Angle():
    x1, y1 = get_erweimaUV(40)
    x2, y2 = get_erweimaUV(36)
    x3, y3 = get_erweimaUV(33)
    x4, y4 = get_erweimaUV(46)
    if x1 == 0 or x2 == 0 or x3 == 0 or x4 == 0:
        return 0
    tan1_2 = (x1-x2)/(y1-y2)
    print(tan1_2)
    # x3=291.244
    # y3=251.81
    # x4=303.2434
    # y4=-126.1871
    #tan3_4 =-(x3-x4)/(y3-y4)
    tan3_4 = (x3-x4)/(y3-y4)
    print(tan3_4)
    tancha = (tan1_2-tan3_4)/(1+tan1_2*tan3_4)
    print(tancha)
    return tancha


def get_zhifangtu(u_start, v_start, u_len, v_len):
    print("hello")


def printhelloword():
    print("hello,word")
    return


if __name__ == '__main__':
    cap0 = cv2.VideoCapture(0)
    cap0.set(3, 3840)
    cap0.set(4, 2160)
    cap0.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap0.set(cv2.CAP_PROP_EXPOSURE, -2)
    # cap0.set(cv2.CAP_PROP_AUTO_EXPOSURE,0.25)
    # cap0.set(cv2.CAP_PROP_EXPOSURE,-1)
    while True:

        ret0, frame0 = cap0.read()
        img0_rectified = cv2.remap(
            frame0, camera_configs.left_map1, camera_configs.left_map2, cv2.INTER_LINEAR)
        corners0, ids0, rejected0 = cv2.aruco.detectMarkers(
            img0_rectified, dictionary, parameters=params)
        kp = np.array(corners0)
        NumofMarks = int(kp.size/8)
        if NumofMarks != 0:
            ids = ids0.flatten().tolist()
            #print ids
            a = 0
            while a < NumofMarks:

                #u,v = np.sum(kp[ids.index(20)], axis=1).flatten() / 4
                u, v = np.sum(kp[a], axis=1).flatten() / 4
                cv2.line(img0_rectified, (int(u-MARK_LEN), int(v-MARK_LEN)),
                         (int(u+MARK_LEN), int(v-MARK_LEN)), (255, 255, 0), 5)
                cv2.line(img0_rectified, (int(u-MARK_LEN), int(v+MARK_LEN)),
                         (int(u+MARK_LEN), int(v+MARK_LEN)), (255, 255, 0), 5)
                cv2.line(img0_rectified, (int(u-MARK_LEN), int(v-MARK_LEN)),
                         (int(u-MARK_LEN), int(v+MARK_LEN)), (255, 255, 0), 5)
                cv2.line(img0_rectified, (int(u+MARK_LEN), int(v-MARK_LEN)),
                         (int(u+MARK_LEN), int(v+MARK_LEN)), (255, 255, 0), 5)
                #text = str(u-1920)+","+str(v-1080)
                text = str(u)+","+str(v)
                cv2.putText(img0_rectified, text, (int(u+30), int(v)),
                            cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 0), 2)
                a = a+1
        ##print u ,v
        #print v

        #print kp[ids.index(47)][1][1]-kp[ids.index(47)][0][1]
        cv2.namedWindow("right", 0)
        cv2.resizeWindow("right", 2048, 1152)
        cv2.imshow("right", img0_rectified)
        cv2.setMouseCallback("right", printxy)

        A = [[u1, v1, 1], [u2, v2, 1], [u3, v3, 1]]
        B = [x1, x2, x3]
        C = [y1, y2, y3]
        A = np.array(A)
        B = np.array(B)
        X = np.linalg.solve(A, B)
        Y = np.linalg.solve(A, C)
        u = 1832
        v = 287
        x = X[0]*u+X[1]*v+X[2]
        y = Y[0]*u+Y[1]*v+Y[2]
        # print(x,y)

        # print "-----------"
        # print X
        # print Y

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
    cap0.release()
    cv2.destroyAllWindows()
