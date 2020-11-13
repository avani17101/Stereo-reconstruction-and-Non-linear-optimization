from __future__ import print_function
import os
import numpy as np
from sklearn.preprocessing import normalize
import cv2
import matplotlib.pyplot as plt
import open3d as o3d

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    #github lol
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')


def extract(file):
    transforms = []
    for i in range(0,21):
        line = file.readline()
        line = line.split()
        arr = np.array(line)
        arr = arr.reshape(3,4)
        transforms.append(arr)
        #Reads the poses text file.
    return transforms

txtfile = open("poses.txt", "r")
transforms = extract(txtfile)
txtfile.close()


def make_new_pt(pt_3d, C):
    # Funtion to convert to 3d points in the world frame.
    P = np.array([
        [C[0][0], C[0][1], C[0][2], C[0][3]],
        [C[1][0], C[1][1], C[1][2], C[1][3]],
        [C[2][0], C[2][1], C[2][2], C[2][3]],
    ]).astype(np.float32)
    pt = np.mat([pt_3d[0], pt_3d[1], pt_3d[2]]).T.astype(np.float32)
    rotation = np.array([[0, 0 , 1],
                        [ -1, 0,  0],
                        [0,  -1, 0]])
    mat = np.matmul(rotation, pt)
    #print(rotatedPoint.shape)
    toPose = np.array([mat[0], mat[1], mat[2], 1])
    rotatedPoint = np.matmul(P, toPose.T)
    #toPose = np.array([rotatedPoint.T, 1]).T
    #print(toPose.shape)
    #exit()
    #Aligning the coordinate frame of open3d with that of cv2
    return [rotatedPoint, rotatedPoint[0]]
    #return [mat, mat[2]]

def main():
    final_output = []
    final_color = []

    window_size = 5
    left_matcher = cv2.StereoSGBM_create(
        minDisparity = -39,
        numDisparities = 144,
        blockSize = 5,
        P1 = 8 * 3 * window_size ** 2,
        P2 = 64 * 3 * window_size ** 2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32,
        preFilterCap = 63
        )

    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    wls_filter.setLambda(80000)
    wls_filter.setSigmaColor(1.3)

    for i in range(60,81):
        imgL = cv2.imread( "./img2/img2/" + "00000004" + str(i) + ".png" , 0)
        imgR = cv2.imread( "./img3/img3/" + "00000004" + str(i) + ".png"  , 0)
        imgL_c = cv2.imread("img2/img2/" + "00000004" + str(i) + ".png" )


        left_disp = left_matcher.compute(imgL,imgR).astype(np.float32)
        right_disp = right_matcher.compute(imgR,imgL).astype(np.float32)
        left_disp = np.int16(left_disp)
        right_disp = np.int16(right_disp)

        Img_Filtered = wls_filter.filter(left_disp, imgL, None, right_disp)
        Img_Filtered = cv2.normalize(src=Img_Filtered, dst=Img_Filtered, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX).astype(np.float32);

        Img_Filtered = cv2.normalize(src = left_disp, dst = Img_Filtered, beta= 0, alpha = 255, norm_type = cv2.NORM_MINMAX).astype(np.float32)

        Img_Filtered = np.float32(Img_Filtered)/16.0
        #imgplot = plt.imshow(Img_Filtered)
        #plt.show()

        calib = np.array([ 7.215377000000e+02, 0.000000000000e+00, 6.095593000000e+02, 4.485728000000e+01,
                           0.000000000000e+00, 7.215377000000e+02, 1.728540000000e+02, 2.163791000000e-01,
                           0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00,2.745884000000e-03])

        P0 = calib.reshape((3,4))

        P0 = np.array([7.070912e+02, 0.000000e+00, 6.018873e+02, 0.000000e+00, 7.070912e+02, 1.831104e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00])
        P0 = P0.reshape((3,3))

        h, w = imgL.shape[:2]
        f = P0[0][0]
        #B = 0.54
        B = 0.53790448812
        Q = np.float32([[1, 0, 0, -P0[0][2]],
                        [0, 1, 0, -P0[1][2]], # turn points 180 deg around x-axis,
                        [0, 0, 0,  P0[0][0]], # so that y-axis looks up
                        [0, 0, -1/B,     0]])

        #Baseline matrix
        #points = cv2.reprojectImageTo3D(Img_Filtered, Q, handleMissingValues = 1)
        imgDim = imgL.shape
        points = np.zeros((imgDim[0], imgDim[1],3))
        rotation = np.array([[-1, 0 , 0],
                             [ 0, -1, 0],
                             [ 0,  0, 1]])

        for x in range(imgDim[0]):
            for y in range(imgDim[1]):
                reprojectedPoint = Q @ np.array([x, y, Img_Filtered[x][y], 1]).T
                reprojectedPoint = reprojectedPoint / reprojectedPoint[3]
                nonHomoPoint = reprojectedPoint[:3]
                #reprojectedPoint = rotation @ nonHomoPoint
                #print(reprojectedPoint.shape)
                #exit()
                #BlankBlank.append()
                #points[x][y] = reprojectedPoint
                points[x][y] = nonHomoPoint

        colors = cv2.cvtColor(imgL_c, cv2.COLOR_BGR2RGB)
        mask = Img_Filtered > Img_Filtered[0][0]
        out_colors = colors[mask]
        out_points = points[mask]
        print(i)
        min = 99999
        max = -9999
        d = 0
        t = 0

        for j,pt in enumerate(out_points):
            t += 1
            if j%5 == 0:
                pointProperties = make_new_pt(pt, transforms[i - 60])
                point3D = pointProperties[0].T
                depthWorldFrame = pointProperties[1]
                d += depthWorldFrame
                if(depthWorldFrame < min):
                    min = depthWorldFrame
                elif depthWorldFrame > max:
                    max = depthWorldFrame
                #Multiplying the reprojected points with the odometry poses to get the points in the world coordinate frame
                #The make new point returns the reprojected, transformed point as well as a paramter used for depth rejection to obtain a clean point cloud.
                #if depthWorldFrame >=50 and depthWorldFrame <=160:
                if depthWorldFrame >= -200 and depthWorldFrame <= 69:
                    final_output.append(point3D)
                    final_color.append(out_colors[j])
        avgD = d // t
        print(f"max {max}, avg {avgD}, min {min}")

    out_fn = "finalRegistration.ply"
    final_output = np.array(final_output)
    final_color = np.array(final_color)
    print(final_output.shape)
    print(final_color.shape)
    write_ply(out_fn, final_output, final_color)

main()

pcd = o3d.io.read_point_cloud("./finalRegistration.ply")
o3d.visualization.draw_geometries([pcd])
