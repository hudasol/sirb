import numpy as np
import pytest

from sirb.downwash import DownwashRule, report, violating_pairs
from sirb.trajectory import Fleet, Segment, Trajectory


def _fleet(dz):
    def hold(p):
        return Trajectory([Segment(0.0, 5.0, np.array([p], float))])
    return Fleet([hold([0, 0, 10]), hold([0, 0, 10 + dz])])


def test_vertical_offset_decides_the_verdict():
    rule = DownwashRule(2.0, 4.0)
    assert violating_pairs(_fleet(3.0), rule)        # 3 m above: inside the 4 m vertical radius
    assert not violating_pairs(_fleet(4.5), rule)
    assert report(_fleet(4.5), rule)["verdict"] == "PASS"


def test_spherical_rule_is_the_ordinary_check():
    assert not violating_pairs(_fleet(2.5), DownwashRule(2.0, 2.0))
    assert violating_pairs(_fleet(1.5), DownwashRule(2.0, 2.0))


def test_rule_validates():
    with pytest.raises(ValueError):
        DownwashRule(2.0, 1.0)
