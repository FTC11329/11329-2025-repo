package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Size;
import org.opencv.android.Utils;
import org.opencv.imgproc.Moments;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Attempt90 {

/*
    RobotSideEnum robotSideEnum;

    private Limelight3A limelight;

    private final double cameraYOffset = 1.1;
    private final double cameraXOffset = Math.PI - 1;

    MatOfPoint3f samplePoints = new MatOfPoint3f(
            new Point3(-1, -1, 0),
            new Point3(1, -1, 0),
            new Point3(1, 1, 0),
            new Point3(-1, 1, 0)
    );

    //define the height that the center of the camera lens is off the ground
    double height = 10;
    //define the angle that the camera is pointing (90 deg = directly forward)
    double cameraAngle = Math.toRadians(62.6);
    // define the range of blocks that the robot can grab inches
    double yMaxExtension = 28.0;
    double xMaxTurn = 18.0;

    //find the corresponding angles that the camera would need to get to go outside the acceptable range
    double maxYangle = (Math.atan(yMaxExtension / height) - cameraAngle);

    //the xMaxRotation cannot be calculated because we do not have the distance of the block

    public Attempt90(HardwareMap hardwareMap, RobotSideEnum robotSideEnum) {
        this.robotSideEnum = robotSideEnum;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    /*public Mat getFrame(){
        URL url = new URL("http://<limelight-ip>:5800/stream.jpg");
        HttpURLConnection conn = (HttpURLConnection) url.openConnection();
        InputStream in = conn.getInputStream();
        Bitmap bmp = BitmapFactory.decodeStream(in);
        Mat frame = new Mat();
        Utils.bitmapToMat(bmp, frame);
    }*/

    /*public Mat transformPerspective(Mat input, Mat K) {
        Size size = input.size();

        // Build rotation matrix around X axis (pitch down)
        Mat R = Mat.eye(3, 3, CvType.CV_64F);
        R.put(1, 1, Math.cos(cameraAngle));
        R.put(1, 2, -Math.sin(cameraAngle));
        R.put(2, 1, Math.sin(cameraAngle));
        R.put(2, 2, Math.cos(cameraAngle));
        // Translation vector (camera at height above ground, looking down)
        Mat t = new MatOfDouble(0, 0, -height); // negative Z

        // Homography: H = K * [R | t] for ground plane Z = 0
        Mat Rt = new Mat(3, 3, CvType.CV_64F);
        Core.gemm(K, R, 1, new Mat(), 0, Rt); // K * R

        // Scale rows so that homography maps to pixel scale (you can tune this)
        double scale = 300.0; // pixels per meter (adjust to your desired field scale)
        Mat scaleMat = Mat.eye(3, 3, CvType.CV_64F);
        scaleMat.put(0, 0, scale);
        scaleMat.put(1, 1, scale);

        Mat H = new Mat();
        Core.gemm(scaleMat, Rt.inv(), 1, new Mat(), 0, H); // invert to get ground-to-pixel

        // Warp the input image using the homography
        Mat output = new Mat();
        Imgproc.warpPerspective(input, output, H, size);

        return output;
    }

    public void proccessImage(){
        Mat img = null;// = getFrame();
        Mat output = new Mat();
        Mat homography = new Mat();
        Imgproc.warpPerspective(img, output, homography, img.size());
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(output, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Pose2D target = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        double minLoss = Double.MAX_VALUE;
        for (MatOfPoint contour : contours) {
            MatOfPoint2f curve = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f polygon = new MatOfPoint2f();
            Imgproc.approxPolyN(curve, polygon, 6);

            //Sort
            Point[] sortedImgPts = sortPointsClockwise(polygon.toArray());
            MatOfPoint2f imagePoints = new MatOfPoint2f(sortedImgPts);

            // Camera intrinsics - dummy
            Mat cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
            cameraMatrix.put(0, 0, 800); // fx
            cameraMatrix.put(1, 1, 800); // fy
            cameraMatrix.put(0, 2, 320); // cx
            cameraMatrix.put(1, 2, 240); // cy

            MatOfDouble distCoefficients = new MatOfDouble(0, 0, 0, 0);

            // Solve PnP
            Mat rot = new Mat();
            Mat pos = new Mat();
            boolean solved = Calib3d.solvePnP(samplePoints, imagePoints, cameraMatrix, distCoefficients, rot, pos);
            Moments m = Imgproc.moments(contour);
            if (m.get_m00() != 0) {
                double cx = m.get_m10() / m.get_m00();
                double cy = m.get_m01() / m.get_m00();
                Pose2D pose = new Pose2D(DistanceUnit.INCH, cx, cy, AngleUnit.DEGREES, 0);
                if (pose.getY(DistanceUnit.INCH) < (Constants.Intake.maxSlidePos + 100) * (1/Constants.Intake.inchToTick)) {
                    double loss = getLoss(pose);
                    if (loss < minLoss) {
                        minLoss = loss;
                        target = pose;
                    }
                }
            }
        }
    }

    private static Point[] sortPointsClockwise(Point[] pts) {
        List<Point> list = Arrays.asList(pts);
        Point center = new Point(0, 0);
        for (Point p : pts) {
            center.x += p.x;
            center.y += p.y;
        }
        center.x /= pts.length;
        center.y /= pts.length;

        list.sort((a, b) -> {
            double angleA = Math.atan2(a.y - center.y, a.x - center.x);
            double angleB = Math.atan2(b.y - center.y, b.x - center.x);
            return Double.compare(angleA, angleB);
        });

        return list.toArray(new Point[0]);
    }

    public double getLoss(Pose2D coords){
        return Math.hypot(coords.getX(DistanceUnit.INCH), coords.getY(DistanceUnit.INCH));
    }*/
}
