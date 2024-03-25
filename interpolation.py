from math import floor

class Interpolate:
    def __init__(self, values):
        self.values = values
        self.n = len(values)
    

    def interpolation_lineaire_3d(self, x, y, t):
        """
        Interpolation linéaire en trois dimensions entre deux points (x0, y0, z0) et (x1, y1, z1) selon un paramètre t.

        Arguments :
        - x0, y0, z0 : coordonnées du premier point
        - x1, y1, z1 : coordonnées du deuxième point
        - t : paramètre d'interpolation (compris entre 0 et 1)

        Sortie :
        - Les coordonnées interpolées (x, y, z) entre les deux points en fonction de t
        """
        x0,y0,z0 = x
        x1,y1,z1 = y

        xr = x0 + (x1 - x0) * t
        yr = y0 + (y1 - y0) * t
        zr = z0 + (z1 - z0) * t


        return xr, yr, zr


    def lerp(self, t):

        i_f = t*(self.n-1)
        t_local = i_f - floor(i_f)

        
        #Limit case
        if (floor(i_f) + 1) >= self.n:
            return self.values[-1]

        # print((floor(i_f)), t)

        return self.interpolation_lineaire_3d(self.values[floor(i_f)], self.values[floor(i_f) + 1], t_local)

        
