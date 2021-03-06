import rospkg
import os.path
import csv
import operator
import numpy as np
from itertools import product
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from sklearn.gaussian_process.kernels import (
    RBF, Matern, RationalQuadratic, ExpSineSquared, DotProduct, ConstantKernel)
from sklearn.preprocessing import normalize

rospack = rospkg.RosPack()
input_file = rospack.get_path("gcount")+"/gcounts_neu.csv"
output_file = rospack.get_path("gcount")+"/gauss_out.csv"
filename = input_file

print("Initializing gaussian kernel")
kernels = [1.0 * RBF(length_scale=1.0, length_scale_bounds=(1e-1, 10.0)),
           1.0 * RationalQuadratic(length_scale=1.0, alpha=0.1),
           1.0 * ExpSineSquared(length_scale=1.0, periodicity=3.0,
                                length_scale_bounds=(0.1, 10.0),
                                periodicity_bounds=(1.0, 10.0)),
           ConstantKernel(0.1, (0.01, 10.0))
           * (DotProduct(sigma_0=1.0, sigma_0_bounds=(0.1, 10.0)) ** 2),
           1.0 * Matern(length_scale=1.0, length_scale_bounds=(1e-1, 10.0),
                        nu=1.5)]



if (os.path.isfile(input_file)):
    with open(filename, 'rU') as p:
        raster_size = 0.1
        raster_add = 1
        my_list = [list(map(float, rec)) for rec in csv.reader(p, delimiter=',')]
        sortedlist = sorted(my_list, key=operator.itemgetter(0), reverse=False)
        sortedlist_y = sorted(my_list, key=operator.itemgetter(1), reverse=False)
    
        #convert list to array
        sorted_arr = np.asarray(sortedlist)
        sorted_arr_y = np.asarray(sortedlist_y)
        #get size of array
        rows = len(sorted_arr)
        #initialize 2d array
        coordinates = np.zeros(shape=(rows, 2), dtype=float)
        #fill 2d array with x,y
        i = 0
        coordinates_old_x = 0
        coordinates_old_y = 0
    
        values = np.zeros(rows)
        while i < rows:
            coordinates[i][0] = sorted_arr[i][0]
            coordinates[i][1] = sorted_arr[i][1]
            values[i] = sorted_arr[i][2]
            i += 1
        # calculate absolute max difference in x and y coordinates
        length = len(sorted_arr)
        min_x = sorted_arr[0][0]
        max_x = sorted_arr[length - 1][0]
        distance_x = 0
    
        print("min value x: ", min_x)
        print("max value x: ", max_x)
    
        if min_x > 0 and max_x > 0:
            distance_x = max_x - min_x
            print"distance pos pos: ", distance_x
        elif min_x < 0 and max_x < 0:
            distance_x = min_x - max_x
            print"distance neg neg: ", distance_x
        elif min_x > 0 and max_x < 0:
            distance_x = max_x - min_x
            print"distance pos neg: ", distance_x
        elif min_x < 0 and max_x > 0:
            distance_x = abs(min_x - max_x)
            print"distance pos neg: ", distance_x
    
        min_y = sorted_arr_y[0][1]
        max_y = sorted_arr_y[length - 1][1]
        distance_y = 0
    
        print"min value y: ", min_y
        print"max value y: ", max_y
    
        if min_y > 0 and max_y > 0:
            distance_y = max_y - min_y
            print"distance pos pos: ", distance_y
        elif min_y < 0 and max_y < 0:
            distance_y = min_y - max_y
            print"distance neg neg: ", distance_y
        elif min_x > 0 and max_y < 0:
            distance_y = max_y - min_y
            print"distance pos neg: ", distance_y
        elif min_y < 0 and max_y > 0:
            distance_y = abs(min_y - max_y)
            print"distance pos neg: ", distance_y
    
        # calculate resize value; assumption pixel=0.1m; round an convert to int
        resize_x = int(round(distance_x / raster_size))
        print"resize_x: ", resize_x
        resize_y = int(round(distance_y / raster_size))
        print"resize_y: ", resize_y
    
        width = int(resize_x+2*raster_add/raster_size)
        heigth = int(resize_y+2*raster_add/raster_size)
    
        #do gauss
        # Input space
        x1 = np.linspace(coordinates[:, 0].min()-raster_add,
                         coordinates[:, 0].max()+raster_add, width)  # p
        x2 = np.linspace(coordinates[:, 1].min()-raster_add,
                         coordinates[:, 1].max()+raster_add, heigth)  # q
        x = (np.array([x1, x2])).T
    
        gp = GaussianProcessRegressor(
            kernel=kernels[1], n_restarts_optimizer=15, normalize_y=True)
        gp.fit(coordinates, values)
        x1x2 = np.array(list(product(x1, x2)))
        y_pred, MSE = gp.predict(x1x2, return_std=True)
    
        X0p, X1p = x1x2[:, 0].reshape(
            width, heigth), x1x2[:, 1].reshape(width, heigth)
        Zp = np.reshape(y_pred, (width, heigth))
    
        # alternative way to generate equivalent X0p, X1p, Zp
        # X0p, X1p = np.meshgrid(x1, x2)
        # Zp = [gp.predict([(X0p[i, j], X1p[i, j]) for i in range(X0p.shape[0])]) for j in range(X0p.shape[1])]
        # Zp = np.array(Zp).T
    
        #fig size in inches
        fig = plt.figure(figsize=(8, 8))
    
        ax = fig.add_subplot(111)
        im = ax.pcolormesh(X0p, X1p, Zp)
        fig.colorbar(im, ax=ax)
    
        #normalize Z
        Zpmax, Zpmin = Zp.max(), Zp.min()
        Zpnorm = (Zp - Zpmin)/(Zpmax - Zpmin)
    
        with open(output_file, mode='w') as gauss_file:
            gauss_writer = csv.writer(gauss_file, delimiter=',')
            for j in range(0, heigth):
                for g in range(0, width):
                    gauss_writer.writerow(
                        [X0p[g][0], X1p[0][j], Zp[g][j], Zpnorm[g][j]])
    
        print"Gaussian Done"
        test = 0
else:
    print("No such file: %s", input_file)