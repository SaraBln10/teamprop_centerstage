package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeampropDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        CENTER,
        NOT_FOUND
    }
    private Location location;
//cadranele st-c-dr
    static final Rect LEFT_ROI = new Rect(
            new Point(20, 35), //60,35
            new Point(80, 175));//120,75
    static final Rect CENTER_ROI = new Rect(
            new Point(80, 35),
            new Point(200, 175));
    static final Rect RIGHT_ROI = new Rect(
            new Point(200, 35), //140,35
            new Point(280, 175)); //200,75
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public TeampropDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(23, 50, 70); //pt galben, dar mai trebuie sa calculez pt rosu, albastru
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI); //submatricea din frame
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Center raw value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean propLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean propCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean propRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (propLeft && propRight && propCenter) {
            location = Location.NOT_FOUND;
            telemetry.addData("Teamprop Location", "not found");
        }
        else if (propLeft) {
            location = Location.LEFT;
            telemetry.addData("Teamprop Location", "left");
        }
        else if (propCenter) {
            location = Location.CENTER;
            telemetry.addData("Teamprop Location", "center");
        }
        else {
            location = Location.RIGHT;
            telemetry.addData("Teamprop Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorFind = new Scalar(255, 0, 0); //RGB rosu, doar ca sa vad pe dashboard cadranele cand gaseste
        Scalar colorDefault = new Scalar(0, 0, 255); //RGB albastru

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorDefault:colorFind);
        Imgproc.rectangle(mat, CENTER_ROI, location == Location.CENTER? colorDefault:colorFind);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorDefault:colorFind);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
