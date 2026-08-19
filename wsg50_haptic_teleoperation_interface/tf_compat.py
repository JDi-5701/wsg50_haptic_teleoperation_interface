"""The handful of tf_transformations functions this package uses.

ros-foxy-tf-transformations exists in apt but is not installed here, and pip has
no such package, so importing it fails outright. Rather than block on a sudo
apt install, this provides the five functions the teleoperation nodes call,
backed by numpy. If the real package is present it is used instead, so
installing it later changes nothing.

Quaternions are [x, y, z, w] throughout, matching both tf_transformations and
geometry_msgs. Getting that order wrong is silent and produces plausible-looking
nonsense, so it is asserted against a known rotation at import.
"""

import numpy as np

try:  # pragma: no cover - depends on what is installed
    from tf_transformations import (  # noqa: F401
        inverse_matrix,
        quaternion_from_matrix,
        quaternion_inverse,
        quaternion_matrix,
        quaternion_multiply,
    )
    USING_REAL_TF_TRANSFORMATIONS = True

except ImportError:
    USING_REAL_TF_TRANSFORMATIONS = False

    def quaternion_matrix(q):
        """[x,y,z,w] -> 4x4 homogeneous rotation."""
        x, y, z, w = q
        n = x * x + y * y + z * z + w * w
        if n < 1e-12:
            return np.identity(4)
        s = 2.0 / n
        xx, yy, zz = x * x * s, y * y * s, z * z * s
        xy, xz, yz = x * y * s, x * z * s, y * z * s
        wx, wy, wz = w * x * s, w * y * s, w * z * s
        return np.array([
            [1.0 - (yy + zz), xy - wz,         xz + wy,         0.0],
            [xy + wz,         1.0 - (xx + zz), yz - wx,         0.0],
            [xz - wy,         yz + wx,         1.0 - (xx + yy), 0.0],
            [0.0,             0.0,             0.0,             1.0],
        ])

    def quaternion_from_matrix(m):
        """4x4 (or 3x3) rotation -> [x,y,z,w], by Shepperd's largest-term rule."""
        r = np.asarray(m, dtype=np.float64)[:3, :3]
        trace = r[0, 0] + r[1, 1] + r[2, 2]
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (r[2, 1] - r[1, 2]) / s
            y = (r[0, 2] - r[2, 0]) / s
            z = (r[1, 0] - r[0, 1]) / s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s
        return np.array([x, y, z, w])

    def inverse_matrix(m):
        return np.linalg.inv(np.asarray(m, dtype=np.float64))

    def quaternion_inverse(q):
        x, y, z, w = q
        n = x * x + y * y + z * z + w * w
        return np.array([-x / n, -y / n, -z / n, w / n])

    def quaternion_multiply(q1, q0):
        """Hamilton product, in tf_transformations' argument order."""
        x1, y1, z1, w1 = q1
        x0, y0, z0, w0 = q0
        return np.array([
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        ])


def _self_check():
    """A 90 deg turn about z must take +x to +y. Wrong quaternion ordering
    still returns a unit quaternion and a valid-looking matrix, so the only way
    to catch it is to rotate something and look."""
    q = [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]      # +90 deg about z
    m = quaternion_matrix(q)
    got = m[:3, :3] @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(got, [0.0, 1.0, 0.0], atol=1e-9), got

    # Round trip through the matrix must come back to the same rotation.
    back = quaternion_from_matrix(m)
    assert np.allclose(back, q, atol=1e-9) or np.allclose(back, -np.array(q),
                                                          atol=1e-9), back

    # q * q^-1 is the identity rotation.
    ident = quaternion_multiply(q, quaternion_inverse(q))
    assert np.allclose(np.abs(ident), [0.0, 0.0, 0.0, 1.0], atol=1e-9), ident

    # Two 90 deg turns about z make 180 deg, which sends +x to -x.
    m180 = quaternion_matrix(quaternion_multiply(q, q))
    assert np.allclose(m180[:3, :3] @ np.array([1.0, 0.0, 0.0]),
                       [-1.0, 0.0, 0.0], atol=1e-9)


_self_check()
