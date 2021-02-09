#!/usr/bin/env python3

from roblib import *
from scipy.ndimage import gaussian_filter

def generate_terrain_1(centre = (0,0), largeur = 20, longueur = 20, pas = 0.1, penteX = 0.2, penteY = 0.2, offset = -30):
    """Cree un relief sous-marin en forme de bol."""
    x = arange(0, largeur+pas, pas)
    y = arange(0, longueur+pas, pas)
    X, Y = meshgrid(x, y)
    Z = (penteX*(X-centre[0]))**2 + (penteY*(Y-centre[1]))**2
    Z = Z+offset
    Z = np.clip(Z, None, 0)
    return X, Y, Z, (largeur, longueur)

def generate_terrain_2(largeur = 20, longueur = 20, pas = 0.1, centres = [(15,15), (45,45)], 
                                pentes = [20, 20], arrondis = [100, 100], offset = -30):
    """Cree un relief sous-marin a base de pics gaussiens."""
    x = arange(0, largeur+pas, pas)
    y = arange(0, longueur+pas, pas)
    X, Y = meshgrid(x, y)
    
    Z = pentes[0]*exp(-((X-centres[0][0])**2+(Y-centres[0][1])**2)/arrondis[0])
    
    for i in range(1,len(centres)):
        Z += pentes[i]*exp(-((X-centres[i][0])**2+(Y-centres[i][1])**2)/arrondis[i])
    
    Z = Z+offset
    Z = np.clip(Z, None, 0)
    return X, Y, Z, (largeur, longueur)

def generate_terrain_3(largeur = 20, longueur = 20, pas = 0.1, offset = -30, angles_coord = array([[5,15,15],[5,5,15]]),
                                epaisseur = 20, hauteur_relief = 20):    
    """Cree un relief sous-marin a base de cretes rectilignes."""
    x = arange(0, largeur+pas, pas)
    y = arange(0, longueur+pas, pas)
    X, Y = meshgrid(x, y)
    
    nb_angles = len(angles_coord.T)
    
    Z = zeros((len(x), len(y)))
    e = int(epaisseur/pas)//2
    relief_height = hauteur_relief
    
    
    angles_coord = angles_coord/pas
    for angle_index in range(nb_angles):
        angle_x, angle_y = angles_coord.T[angle_index]
        next_angle_x, next_angle_y = angles_coord.T[(angle_index+1)%nb_angles]
        
        for p in linspace(0, 1, 30):
            a = (1-p)*angle_x + p*next_angle_x
            b = (1-p)*angle_y + p*next_angle_y
#            print "#### DEBUG: " , int(a)-e, int(a)+e, int(b)-e, int(b)+e
            begin_x, end_x, begin_y, end_y = int(max(0, a-e)), int(min(Z.shape[0], a+e)), int(max(0, b-e)), int(min(Z.shape[1], b+e))
            Z[begin_x:end_x, begin_y:end_y] = relief_height * ones((end_x-begin_x, end_y-begin_y))
            
    
    
    Z = gaussian_filter(Z+offset, 10)
    Z = np.clip(Z, None, 0)
    return X, Y, Z, (largeur, longueur)

def read_terrain_from_txt(filename):
    pas = 0.5
    Z = np.loadtxt(filename)
    x = np.arange(Z.shape[0])*pas
    y = np.arange(Z.shape[1])*pas    
    X, Y = meshgrid(x, y)
    largeur = (Z.shape[0]-1)*pas
    longueur = (Z.shape[1]-1)*pas
    return X, Y, Z, (largeur, longueur)


if __name__ == "__main__":
    ax = Axes3D(figure())
    
    angles_coord = array([[ 5, 30, 45, 30, 10],
                          [30, 45, 20, 2, 5]])
    Xt, Yt, Zt, real_size = read_terrain_from_txt("data-cycle-stable/mat.txt")
    
    ax.plot_surface(Xt, Yt, Zt, cmap=cm.viridis, linewidth=0, antialiased=False)
    ax.set_aspect('equal')
    ax.set_xlabel('X axis, meters')
    ax.set_ylabel('Y axis, meters')
    ax.set_zlabel('Z axis, meters')
    
    plt.show()
