import pytest
from FCLCollisionInfo import FCLCollisionInfo


class TestFCLCollisionInfo():

    def test_unknown_file_throws(self):

        with pytest.raises(RuntimeError):
            FCLCollisionInfo("this is an illegal file name.stl")
    

    def test_illegal_file_type_throws(self):

        with pytest.raises(RuntimeError):
            FCLCollisionInfo("this is an illegal file name.dae")
        
        with pytest.raises(RuntimeError):
            FCLCollisionInfo("")