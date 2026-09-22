import unittest

import numpy as np

from piper_button_perception.localization import (
    physical_size_ratio, project_pixel, robust_depth, stable_candidate, transform_point,
)


class LocalizationTest(unittest.TestCase):
    def test_robust_depth_rejects_invalid_values(self):
        depth = np.full((9,9),0.30,dtype=np.float32)
        depth[4,4] = np.nan
        depth[3,3] = 5.0
        self.assertAlmostEqual(robust_depth(depth,4,4,2,0.1,1.2),0.30,places=5)

    def test_projection_and_transform(self):
        camera = [200.,0.,160.,0.,200.,120.,0.,0.,1.]
        point = project_pixel(180,100,0.5,camera)
        np.testing.assert_allclose(point,[0.05,-0.05,0.5])
        moved = transform_point(point,[1,2,3],[0,0,0,1])
        np.testing.assert_allclose(moved,[1.05,1.95,3.5])

    def test_physical_size_and_stability_filters(self):
        self.assertAlmostEqual(physical_size_ratio([0,0,40,40],240,0.3,0.05),1.0)
        samples = [
            ("red",np.array([0.30,0.00,0.24]),0.9,(160,120),0.3),
            ("red",np.array([0.301,0.001,0.239]),0.8,(161,120),0.301),
            None,
            ("red",np.array([0.299,-0.001,0.241]),0.85,(160,121),0.299),
        ]
        stable = stable_candidate(samples,3,0.01)
        self.assertEqual(stable[0],"red")
        np.testing.assert_allclose(stable[1],[0.30,0.00,0.24])
        samples[-1] = ("red",np.array([0.36,0,0.24]),0.8,(160,120),0.3)
        self.assertIsNone(stable_candidate(samples,3,0.01))


if __name__ == "__main__":
    unittest.main()
