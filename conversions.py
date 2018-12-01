"""
Code from https://github.com/scivision/pymap3d
"""

from numpy import radians, sin, cos, tan, arctan, hypot, degrees, arctan2, sqrt, pi
import numpy as np
from typing import Tuple

class Ellipsoid:
    """
    generate reference ellipsoid parameters
    https://en.wikibooks.org/wiki/PROJ.4#Spheroid
    """

    def __init__(self, model='wgs84'):
        if model == 'wgs84':
            """https://en.wikipedia.org/wiki/World_Geodetic_System#WGS84"""
            self.a = 6378137.  # semi-major axis [m]
            self.f = 1 / 298.2572235630  # flattening
            self.b = self.a * (1 - self.f)  # semi-minor axis
        elif model == 'grs80':
            """https://en.wikipedia.org/wiki/GRS_80"""
            self.a = 6378137.  # semi-major axis [m]
            self.f = 1 / 298.257222100882711243  # flattening
            self.b = self.a * (1 - self.f)  # semi-minor axis
        elif model == 'clrk66':  # Clarke 1866
            self.a = 6378206.4  # semi-major axis [m]
            self.b = 6356583.8  # semi-minor axis
            self.f = -(self.b / self.a - 1)
        elif model == 'mars':  # https://tharsis.gsfc.nasa.gov/geodesy.html
            self.a = 3396900
            self.b = 3376097.80585952
            self.f = 1 / 163.295274386012
        elif model == 'moon':
            self.a = 1738000.
            self.b = 1738000.
            self.f = 0.
        elif model == 'venus':
            self.a = 6051000.
            self.b = 6051000.
            self.f = 0.
        else:
            raise NotImplementedError('{} model not implemented, let us know and we will add it (or make a pull request)'.format(model))


def get_radius_normal(lat_radians, ell):
    """ Compute normal radius of planetary body"""
    if ell is None:
        ell = Ellipsoid()

    a = ell.a
    b = ell.b

    return a**2 / sqrt(a**2 * cos(lat_radians)**2 + b**2 * sin(lat_radians)**2)


def ecef2ned(x, y, z, lat0, lon0, h0, ell=None, deg=True):
    """
    input
    -----
    x,y,z  [meters] target ECEF location                             [0,Infinity)
    Observer: lat0, lon0, h0 (altitude, meters)
    ell    reference ellipsoid
    deg    degrees input/output  (False: radians in/out)
    output:
    -------
    n,e,d  North,east, down [m]
    """
    e, n, u = ecef2enu(x, y, z, lat0, lon0, h0, ell, deg=deg)

    return n, e, -u

def ecef2enu(x, y, z, lat0, lon0, h0, ell, deg):
    """
    input
    -----
    x,y,z  [meters] target ECEF location                             [0,Infinity)
    Observer: lat0, lon0, h0 (altitude, meters)
    ell    reference ellipsoid
    deg    degrees input/output  (False: radians in/out)
    output:
    -------
    e,n,u   East, North, Up [m]
    """
    x0, y0, z0 = geodetic2ecef(lat0, lon0, h0, ell, deg=deg)

    return uvw2enu(x - x0, y - y0, z - z0, lat0, lon0, deg=deg)


def uvw2enu(u, v, w, lat0, lon0, deg=True):
    if deg:
        lat0 = radians(lat0)
        lon0 = radians(lon0)

    t = cos(lon0) * u + sin(lon0) * v
    East = -sin(lon0) * u + cos(lon0) * v
    Up = cos(lat0) * t + sin(lat0) * w
    North = -sin(lat0) * t + cos(lat0) * w

    return East, North, Up


def geodetic2ecef(lat, lon, alt, ell=None, deg=True):
    """
    Point
    input:
    -----
    lat, lon (degrees)
    alt (altitude, meters)    [0, Infinity)
    ell    reference ellipsoid
    deg    degrees input/output  (False: radians in/out)
    output: ECEF x,y,z (meters)
    """
    if ell is None:
        ell = Ellipsoid()

    if deg:
        lat = radians(lat)
        lon = radians(lon)

    with np.errstate(invalid='ignore'):
        # need np.any() to handle scalar and array cases
        if np.any((lat < -pi / 2) | (lat > pi / 2)):
            raise ValueError('-90 <= lat <= 90')

        if np.any((lon < -pi) | (lon > 2 * pi)):
            raise ValueError('-180 <= lat <= 360')

        if np.any(np.asarray(alt) < 0):
            raise ValueError('altitude  [0, Infinity)')
    # radius of curvature of the prime vertical section
    N = get_radius_normal(lat, ell)
    # Compute cartesian (geocentric) coordinates given  (curvilinear) geodetic
    # coordinates.
    x = (N + alt) * cos(lat) * cos(lon)
    y = (N + alt) * cos(lat) * sin(lon)
    z = (N * (ell.b / ell.a)**2 + alt) * sin(lat)

    return x, y, z


def ned2geodetic(n, e, d, lat0, lon0, h0, ell=None, deg=True):
    """
    input
    -----
    n,e,d   North, east, down (meters)
    Observer: lat0, lon0, h0 (altitude, meters)
    ell    reference ellipsoid
    deg    degrees input/output  (False: radians in/out)
    output:
    -------
    target: lat,lon, h  (degrees/radians,degrees/radians, meters)
    """
    x, y, z = enu2ecef(e, n, -d, lat0, lon0, h0, ell, deg=deg)

    return ecef2geodetic(x, y, z, ell, deg=deg)


def ecef2geodetic(x, y, z, ell=None, deg=True):
    """
    convert ECEF (meters) to geodetic coordinates
    input
    -----
    x,y,z  [meters] target ECEF location                             [0,Infinity)
    ell    reference ellipsoid
    deg    degrees input/output  (False: radians in/out)
    output
    ------
    lat,lon   (degrees/radians)
    alt  (meters)
    based on:
    You, Rey-Jer. (2000). Transformation of Cartesian to Geodetic Coordinates without Iterations.
    Journal of Surveying Engineering. doi: 10.1061/(ASCE)0733-9453
    """
    if ell is None:
        ell = Ellipsoid()

    r = sqrt(x**2 + y**2 + z**2)

    E = sqrt(ell.a**2 - ell.b**2)

    # eqn. 4a
    u = sqrt(0.5 * (r**2 - E**2) + 0.5 * sqrt((r**2 - E**2)**2 + 4 * E**2 * z**2))

    Q = hypot(x, y)

    huE = hypot(u, E)

    # eqn. 4b
    Beta = arctan(huE / u * z / hypot(x, y))

    # eqn. 13
    eps = ((ell.b * u - ell.a * huE + E**2) * sin(Beta)) / (ell.a * huE * 1 / cos(Beta) - E**2 * cos(Beta))

    Beta += eps
# %% final output
    lat = arctan(ell.a / ell.b * tan(Beta))

    lon = arctan2(y, x)

    # eqn. 7
    alt = sqrt((z - ell.b * sin(Beta))**2 + (Q - ell.a * cos(Beta))**2)

    if deg:
        return degrees(lat), degrees(lon), alt
    else:
        return lat, lon, alt  # radians


def enu2ecef(e1, n1, u1, lat0, lon0, h0, ell=None, deg=True):
    """
    ENU to ECEF
    inputs:
     e1, n1, u1 (meters)   east, north, up
     observer: lat0, lon0, h0 (degrees/radians,degrees/radians, meters)
    ell    reference ellipsoid
    deg    degrees input/output  (False: radians in/out)
    output
    ------
    x,y,z  [meters] target ECEF location                         [0,Infinity)
    """
    x0, y0, z0 = geodetic2ecef(lat0, lon0, h0, ell, deg=deg)
    dx, dy, dz = enu2uvw(e1, n1, u1, lat0, lon0, deg=deg)


def enu2uvw(east, north, up, lat0, lon0, deg=True):
    if deg:
        lat0 = radians(lat0)
        lon0 = radians(lon0)

    t = cos(lat0) * up - sin(lat0) * north
    w = sin(lat0) * up + cos(lat0) * north

    u = cos(lon0) * t - sin(lon0) * east
    v = sin(lon0) * t + cos(lon0) * east

    return u, v, w
