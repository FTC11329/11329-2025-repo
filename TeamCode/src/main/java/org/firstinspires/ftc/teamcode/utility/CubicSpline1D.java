package org.firstinspires.ftc.teamcode.utility;

import java.util.List;

public class CubicSpline1D {

    private static class Segment {
        double a, b, c, d, t0, t1;
        Segment(double a, double b, double c, double d, double t0, double t1) {
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
            this.t0 = t0;
            this.t1 = t1;
        }
    }

    private Segment[] segments;

    public CubicSpline1D(List<Double> tList, List<Double> yList) {
        int n = tList.size();

        if (n != yList.size() || n < 2) {
            throw new IllegalArgumentException("Time and value lists must be same length and >= 2");
        }

        double[] t = new double[n];
        double[] y = new double[n];

        for (int i = 0; i < n; i++) {
            t[i] = tList.get(i);
            y[i] = yList.get(i);
        }

        int m = n - 1; // number of segments
        segments = new Segment[m];

        double[] h = new double[m];
        for (int i = 0; i < m; i++) {
            h[i] = t[i + 1] - t[i];
        }

        // Step 1: Solve for second derivatives (natural spline)
        double[] alpha = new double[m];
        for (int i = 1; i < m; i++) {
            alpha[i] = (3/h[i]) * (y[i+1] - y[i]) - (3/h[i-1]) * (y[i] - y[i-1]);
        }

        double[] l = new double[n];
        double[] mu = new double[n];
        double[] z = new double[n];

        l[0] = 1;
        mu[0] = 0;
        z[0] = 0;

        for (int i = 1; i < m; i++) {
            l[i] = 2*(t[i+1] - t[i-1]) - h[i-1]*mu[i-1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i-1]*z[i-1]) / l[i];
        }

        l[n-1] = 1;
        z[n-1] = 0;

        double[] c = new double[n];
        double[] b = new double[m];
        double[] d = new double[m];

        c[n-1] = 0;

        // Step 2: Back substitution for c, b, d
        for (int j = m - 1; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j+1];
            b[j] = (y[j+1] - y[j]) / h[j] - h[j] * (c[j+1] + 2*c[j]) / 3;
            d[j] = (c[j+1] - c[j]) / (3 * h[j]);
        }

        // Step 3: Build segments
        for (int i = 0; i < m; i++) {
            segments[i] = new Segment(
                    y[i],      // a
                    b[i],      // b
                    c[i],      // c
                    d[i],      // d
                    t[i], t[i+1]
            );
        }
    }

    /** Evaluate spline at time T */
    public double eval(double T) {
        Segment seg = null;

        // Find segment containing T
        for (Segment s : segments) {
            if (T >= s.t0 && T <= s.t1) {
                seg = s;
                break;
            }
        }

        // Clamp to nearest segment if out of bounds
        if (seg == null) {
            if (T < segments[0].t0) seg = segments[0];
            else seg = segments[segments.length - 1];
        }

        double dt = T - seg.t0;
        return seg.a + seg.b * dt + seg.c * dt * dt + seg.d * dt * dt * dt;
    }
}
