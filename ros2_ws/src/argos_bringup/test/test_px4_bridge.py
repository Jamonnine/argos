"""PX4 브릿지 좌표 변환 테스트."""
import pytest


def enu_to_ned(x, y, z):
    return y, x, -z

def ned_to_enu(x, y, z):
    return y, x, -z


class TestCoordinateConversion:

    def test_enu_to_ned_basic(self):
        """ENU(1,2,3) → NED(2,1,-3)."""
        nx, ny, nz = enu_to_ned(1.0, 2.0, 3.0)
        assert nx == pytest.approx(2.0)  # North = ENU Y
        assert ny == pytest.approx(1.0)  # East = ENU X
        assert nz == pytest.approx(-3.0) # Down = -Up

    def test_ned_to_enu_basic(self):
        """NED(2,1,-3) → ENU(1,2,3)."""
        ex, ey, ez = ned_to_enu(2.0, 1.0, -3.0)
        assert ex == pytest.approx(1.0)
        assert ey == pytest.approx(2.0)
        assert ez == pytest.approx(3.0)

    def test_roundtrip(self):
        """ENU→NED→ENU 왕복 변환 일치."""
        original = (5.5, -3.2, 8.0)
        ned = enu_to_ned(*original)
        back = ned_to_enu(*ned)
        for o, b in zip(original, back):
            assert o == pytest.approx(b)

    def test_zero(self):
        """원점은 변환해도 원점."""
        nx, ny, nz = enu_to_ned(0, 0, 0)
        assert nx == 0 and ny == 0 and nz == 0

    def test_negative_altitude(self):
        """ENU z=-1 (지하) → NED z=1 (아래)."""
        _, _, nz = enu_to_ned(0, 0, -1.0)
        assert nz == pytest.approx(1.0)

    def test_hover_at_8m(self):
        """ARGOS 정찰 고도 8m → NED -8m."""
        _, _, nz = enu_to_ned(0, 0, 8.0)
        assert nz == pytest.approx(-8.0)
