"""Latitude/longitude (WGS84/ETRS89) to UTM with Krueger's series (~mm within the zone).

Default zone 32N (EPSG:25832, the Danish standard). The harbour frame "porto" has its x
axis along grid East and y along grid North of this projection.
"""
import numpy as np

_A = 6378137.0
_F = 1 / 298.257223563
_K0 = 0.9996
_N = _F / (2 - _F)
_AA = _A / (1 + _N) * (1 + _N ** 2 / 4 + _N ** 4 / 64)
_ALPHA = (_N / 2 - 2 * _N ** 2 / 3 + 5 * _N ** 3 / 16,
          13 * _N ** 2 / 48 - 3 * _N ** 3 / 5,
          61 * _N ** 3 / 240)


def latlon_to_utm(lat: float, lon: float, zone: int = 32):
    """Return (East, North) in metres."""
    lat, lon = np.radians(float(lat)), np.radians(float(lon))
    lon0 = np.radians(-183 + 6 * zone)
    e = 2 * np.sqrt(_N) / (1 + _N)
    t = np.sinh(np.arctanh(np.sin(lat)) - e * np.arctanh(e * np.sin(lat)))
    xi_p = np.arctan2(t, np.cos(lon - lon0))
    eta_p = np.arctanh(np.sin(lon - lon0) / np.sqrt(1 + t ** 2))
    xi = xi_p + sum(a * np.sin(2 * j * xi_p) * np.cosh(2 * j * eta_p) for j, a in enumerate(_ALPHA, 1))
    eta = eta_p + sum(a * np.cos(2 * j * xi_p) * np.sinh(2 * j * eta_p) for j, a in enumerate(_ALPHA, 1))
    return 500000 + _K0 * _AA * eta, _K0 * _AA * xi
