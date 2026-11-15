#!/usr/bin/env python3
"""
test_assist.py - unit tests for assist.py on synthetic signals (PLAN_STEADY_HANDS.md, phase 2).

  python test_assist.py          # run all tests, prints measured numbers
  python test_assist.py -v       # verbose

Pass criteria (from the plan):
  GripLimiter on/off                     -> 5 N / 60 N cap
  MotionScaler at 0.3                    -> 100 mm in gives 30 mm out
  MotionScaler scale switch in motion    -> no jump: the output step on the switching frame differs from
                                            new_scale * input step by < 0.1 mm
  JitterFilter, still hand, 10 Hz 1 mm   -> amplitude reduced by >= 70 %
  JitterFilter, moving hand, 0.2 Hz 100 mm -> lag < 80 ms
Standard library only.
"""
import math
import sys
import unittest

from assist import ASSIST_PARAMS, GripLimiter, JitterFilter, MotionScaler

RATE = 83.33
DT = 1.0 / RATE


def run_filter(f, signal_fn, seconds, base=(1.45, -0.03, -0.13)):
    """Feed the x axis with signal_fn(t) (metres), y/z constant. Returns (t, in, out) lists."""
    ts, xin, xout = [], [], []
    for k in range(int(seconds * RATE)):
        t = k * DT
        v = base[0] + signal_fn(t)
        out = f([v, base[1], base[2]], t)
        ts.append(t), xin.append(v), xout.append(out[0])
    return ts, xin, xout


def lag_seconds(xin, xout, max_shift=40):
    """Delay of xout behind xin by cross-correlation with parabolic sub-sample refinement."""
    mi, mo = sum(xin) / len(xin), sum(xout) / len(xout)
    a = [v - mi for v in xin]
    b = [v - mo for v in xout]
    n = len(a) - max_shift

    def corr(k):
        return sum(a[i] * b[i + k] for i in range(n))

    cs = [corr(k) for k in range(max_shift + 1)]
    k = max(range(len(cs)), key=cs.__getitem__)
    if 0 < k < max_shift:
        y0, y1, y2 = cs[k - 1], cs[k], cs[k + 1]
        k = k + 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
    return k * DT


def amplitude(x):
    return (max(x) - min(x)) / 2


class TestGripLimiter(unittest.TestCase):
    def test_caps(self):
        g = GripLimiter()
        self.assertEqual(g.cap(True), 5.0)
        self.assertEqual(g.cap(False), 60.0)
        self.assertEqual(g.forcerange(True), (-60.0, 5.0))   # opening keeps full force
        self.assertEqual(g.forcerange(False), (-60.0, 60.0))
        print(f"\n  GripLimiter: on {g.cap(True)} N, off {g.cap(False)} N")


class TestMotionScaler(unittest.TestCase):
    def test_scale_03(self):
        s = MotionScaler(0.3)
        s.reset([1.0, 0.0, 0.0], [0.5, -0.1, 0.62])
        out = s([1.1, 0.0, 0.0])
        moved_mm = (out[0] - 0.5) * 1000
        print(f"\n  MotionScaler 0.3: 100 mm in -> {moved_mm:.3f} mm out")
        self.assertAlmostEqual(moved_mm, 30.0, places=6)
        self.assertAlmostEqual(out[1], -0.1)
        self.assertAlmostEqual(out[2], 0.62)

    def test_switch_no_jump(self):
        s = MotionScaler(1.0)
        s.reset([0, 0, 0], [0.5, -0.1, 0.62])
        v = 0.10                                   # hand moving at 10 cm/s along x and y
        worst, worst_naive = 0.0, 0.0
        prev_in, prev_out = [0, 0, 0], [0.5, -0.1, 0.62]
        for k in range(1, 200):
            x = [v * k * DT, v * k * DT, 0.0]
            switched = k in (60, 120)
            if switched:
                new = 0.3 if s.scale == 1.0 else 1.0
                # naive (no re-anchor) for comparison
                naive = [o + new * (i - a) for i, a, o in zip(x, s.anchor_in, s.anchor_out)]
                s.set_scale(new)
            out = s(x)
            if switched:
                step = [o - p for o, p in zip(out, prev_out)]
                expect = [s.scale * (i - p) for i, p in zip(x, prev_in)]
                worst = max(worst, max(abs(a - b) for a, b in zip(step, expect)))
                worst_naive = max(worst_naive, max(abs(n - p - e) for n, p, e in zip(naive, prev_out, expect)))
            prev_in, prev_out = x, out
        print(f"\n  MotionScaler switch at 10 cm/s: jump {worst * 1000:.4f} mm "
              f"(without re-anchoring it would be {worst_naive * 1000:.1f} mm)")
        self.assertLess(worst * 1000, 0.1)


class TestJitterFilter(unittest.TestCase):
    def test_still_hand_10hz(self):
        f = JitterFilter()
        ts, xin, xout = run_filter(f, lambda t: 0.001 * math.sin(2 * math.pi * 10 * t), 10)
        skip = int(3 * RATE)
        a_in, a_out = amplitude(xin[skip:]), amplitude(xout[skip:])
        red = 1 - a_out / a_in
        print(f"\n  JitterFilter (min_cutoff {f.min_cutoff} Hz, beta {f.beta}): 10 Hz 1 mm sine "
              f"{a_in * 1000:.3f} -> {a_out * 1000:.3f} mm, reduced {red * 100:.1f} %")
        self.assertGreaterEqual(red, 0.70)

    def test_moving_hand_lag(self):
        f = JitterFilter()
        ts, xin, xout = run_filter(f, lambda t: 0.1 * math.sin(2 * math.pi * 0.2 * t), 40)
        skip = int(10 * RATE)
        lag = lag_seconds(xin[skip:], xout[skip:])
        print(f"\n  JitterFilter (min_cutoff {f.min_cutoff} Hz, beta {f.beta}): 0.2 Hz 100 mm sine, "
              f"lag {lag * 1000:.1f} ms")
        self.assertLess(lag, 0.080)

    def test_duplicate_timestamp_and_gaps(self):
        f = JitterFilter()
        a = f([1, 2, 3], 0.0)
        b = f([1.01, 2, 3], 0.012)
        c = f([5, 5, 5], 0.012)              # same timestamp: ignored
        self.assertEqual(b, c)
        d = f([1.01, 2, 3], 10.0)            # big gap: falls back to the default dt, no blow-up
        self.assertTrue(all(math.isfinite(v) for v in d))
        self.assertEqual(a, [1, 2, 3])

    def test_reference_cutoffs(self):
        """Informational: the plan's numbers for other cutoffs (no assertion on 1 Hz)."""
        print()
        for fc in (1.0, 2.0, 3.0):
            f = JitterFilter(min_cutoff=fc, beta=0.0)
            _, xi, xo = run_filter(f, lambda t: 0.001 * math.sin(2 * math.pi * 10 * t), 10)
            red = 1 - amplitude(xo[250:]) / amplitude(xi[250:])
            f = JitterFilter(min_cutoff=fc, beta=0.0)
            _, xi, xo = run_filter(f, lambda t: 0.1 * math.sin(2 * math.pi * 0.2 * t), 40)
            lag = lag_seconds(xi[833:], xo[833:])
            print(f"  cutoff {fc:.0f} Hz, beta 0: 10 Hz reduced {red * 100:5.1f} %, 0.2 Hz lag {lag * 1000:5.1f} ms")


if __name__ == "__main__":
    print("ASSIST_PARAMS:", ASSIST_PARAMS)
    unittest.main(verbosity=2 if "-v" in sys.argv else 1)
