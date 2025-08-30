package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Vector;

public class PathSpline {
    public CubicSpline1D xSpline;
    public CubicSpline1D ySpline;

    public Pose evaluate(double t){
        return new Pose(xSpline.evaluate(t), ySpline.evaluate(t));
    }

    public Vector velocity(double t){
        Vector v = new Vector();
        v.setOrthogonalComponents(xSpline.velocity(t), ySpline.velocity(t));
        return v;
    }
}
