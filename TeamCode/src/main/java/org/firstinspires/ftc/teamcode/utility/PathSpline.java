package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.MathFunctions;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Vector;

public class PathSpline {
    public CubicSpline1D xSpline;
    public CubicSpline1D ySpline;
    public CubicSpline1D headingSpline;

    public Pose evaluate(double t){
        return new Pose(xSpline.evaluate(t), ySpline.evaluate(t));
    }

    public Vector velocity(double t){
        Vector v = new Vector();
        v.setOrthogonalComponents(xSpline.velocity(t), ySpline.velocity(t));
        return v;
    }

    public Vector acceleration(double t){
        Vector v = new Vector();
        v.setOrthogonalComponents(xSpline.acceleration(t), ySpline.acceleration(t));
        return v;
    }

    public double curvature(double t){
        Vector derivative = velocity(t);
        Vector secondDerivative = acceleration(t);

        if (derivative.getMagnitude() == 0) return 0;
        return (MathFunctions.crossProduct(derivative, secondDerivative))/Math.pow(derivative.getMagnitude(),3);
    }

    public double heading(double t){
        return headingSpline.evaluate(t);
    }
}
