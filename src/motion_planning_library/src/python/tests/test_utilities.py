from typing import Tuple
from transform import Transform
import utilities
from settings import ROOT_FRAME_NAME

from ctypes import util
import pytest
import math
import numpy as np

class TestUtilities:

    def test_euler_to_quaternion_numerical_properties(self):

        roll = math.pi/2
        quaternion = utilities.euler_to_quaternion(roll, 0.,0.)
        delta = 0.0001
        assert quaternion.x == pytest.approx(0.7070727, delta)
        assert quaternion.y == pytest.approx(0., delta)
        assert quaternion.z == pytest.approx(0., delta)
        assert quaternion.w == pytest.approx(0.7071408, delta)

    def test_identity_pose(self):

        identity_pose = utilities.identity_pose()
        delta = 0.0001
        assert identity_pose.position.x == pytest.approx(0., delta)
        assert identity_pose.position.y == pytest.approx(0., delta)
        assert identity_pose.position.z == pytest.approx(0., delta)

        assert identity_pose.orientation.x == pytest.approx(0., delta)
        assert identity_pose.orientation.y == pytest.approx(0., delta)
        assert identity_pose.orientation.z == pytest.approx(0., delta)
        assert identity_pose.orientation.w == pytest.approx(1., delta)


    @pytest.mark.parametrize("transform, expected_output", [
        (Transform("a parent", "a child", 0., 0., 0., 0., 0., 0.), (0.,0.,0.)),
        (Transform("a parent", "a child", 0., 0., 0., math.pi/2, 0., 0.), (math.pi/2,0.,0.)),
        (Transform("a parent", "a child", 0., 0., 0., 0., 1.25, 0.), (0.,1.25,0.)),
        (Transform("a parent", "a child", 0., 0., 0., 0., 0., 0.125), (0.,0.,0.125))
        ])
    def test_converting_rotation_to_euler_angles_numerically(self, transform: Transform, expected_output: Tuple):

        rotation = transform.rotation_matrix()
        result = utilities.rot2euler_numerically(rotation) 

        assert isinstance(result, tuple)
        assert len(result) == 3
        assert result == pytest.approx(expected_output)

   
    def test_converting_rotation_to_euler_angles_symbolically(self):

        transform = Transform("a parent", "a child", 0., 0., 0., math.pi/2, 0., 0.)
        symbolic_transform = transform.to_sympy()
        symbolic_rotation = symbolic_transform[0:3,0:3]

        result = utilities.rot2euler_symbolically(symbolic_rotation) 
        roll, pitch, yaw = result

        assert isinstance(result, tuple)
        assert len(result) == 3
        
        assert roll == pytest.approx(math.pi/2)
        assert pitch == pytest.approx(0.)
        assert yaw == pytest.approx(0.)


    @pytest.mark.parametrize("array1, array2, expected_error", [
        (np.array([0.,0.,0.]), np.array([0.,0.,0.]), 0.),
        (np.array([1.,2.,3.]), np.array([1.,2.,3.]), 0.),
        (np.array([3.5,0.,0.]), np.array([1.,0.,0.]), 2.5 * 2.5)
        ])
    def test_sum_square_errors(self, array1: np.ndarray, array2: np.ndarray, expected_error: float):

        error = utilities.sum_square_error(array1, array2)
        assert error == pytest.approx(expected_error)

    def test_lookup_transform(self):

        transform1 = Transform(ROOT_FRAME_NAME, "child1", 0.,0.,0.,0.,0.,0.)
        transform2 = Transform("child1", "child2", 0.,0.,0.,0.,0.,0.)
        transform3 = Transform("child2", "child3", 0.,0.,0.,0.,0.,0.)
        transforms = [transform1, transform2, transform3]

        parent = utilities.lookup_parent_transform(transforms, "child3")
        assert parent == transform3

    def test_lookup_transform_without_parent_raises_exception(self):
        
        transform1 = Transform(ROOT_FRAME_NAME, "child1", 0.,0.,0.,0.,0.,0.)
        transforms = [transform1]

        with pytest.raises(RuntimeError):
            parent = utilities.lookup_parent_transform(transforms, "this frame DNE")
        
        with pytest.raises(RuntimeError):
            parent = utilities.lookup_parent_transform(transforms + transforms, "child1")

        with pytest.raises(RuntimeError):
            parent = utilities.lookup_parent_transform([], "child1")