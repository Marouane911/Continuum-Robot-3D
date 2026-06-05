import numpy as np
from scipy.spatial.transform import Rotation as Rot


class CTRData:

    def __init__(self, matrix_data, iEnd, S):

        self.matrix_data = matrix_data
        self.iEnd = iEnd
        self.S = S

        self.num_nodes = matrix_data.shape[1]

        self.x = matrix_data[0]
        self.y = matrix_data[1]
        self.z = matrix_data[2]

        xyz = matrix_data[0:3, :].T

        dxyz = np.diff(xyz, axis=0)

        dl = np.linalg.norm(dxyz, axis=1)

        self.length_axis = np.concatenate(
            ([0], np.cumsum(dl))
        )

        self.s_ext = S[iEnd[2]]
        self.s_mid = S[iEnd[1]]

        self.end_ext = (
            np.argmin(
                np.abs(self.length_axis - self.s_ext)
            ) + 1
        )

        self.end_mid = (
            np.argmin(
                np.abs(self.length_axis - self.s_mid)
            ) + 1
        )

        self.end_int = self.num_nodes

        self.tip_x = self.x[-1]
        self.tip_y = self.y[-1]
        self.tip_z = self.z[-1]

        self.R_tip = matrix_data[3:12, -1].reshape(
            (3, 3),
            order="C"
        )

        rotation = Rot.from_matrix(
            self.R_tip
        )

        (
            self.tip_angle_x,
            self.tip_angle_y,
            self.tip_angle_z
        ) = rotation.as_euler(
            "xyz",
            degrees=True
        )

        self.theta_2 = matrix_data[17]
        self.theta_3 = matrix_data[18]