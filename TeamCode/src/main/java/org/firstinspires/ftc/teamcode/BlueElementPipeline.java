package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueElementPipeline extends OpenCvPipeline {

    public static int L1 = 101, L2 = 7, L3 = 13, H1 = 150, H2 = 166, H3 = 201;

    public double positionx, positiony;

    public double area;
    @Override
    public Mat processFrame(Mat input) {
        Mat hsvFrame = new Mat();
        Mat thresholdedFrame = new Mat();
        Mat gFrame = new Mat();



        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);
        //Imgproc.cvtColor(input, gFrame, Imgproc.COLOR_RGB2GRAY);

        // Define the white color range in HSV
        Scalar lowerWhite = new Scalar(L1, L2, L3);
        Scalar upperWhite = new Scalar(H1, H2, H3);

        // Threshold the frame to detect white regions

        //Core.inRange(gFrame, lowerWhite, upperWhite, gFrame);
        Imgproc.morphologyEx(hsvFrame, hsvFrame, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9, 9)));
        Core.inRange(hsvFrame, lowerWhite, upperWhite, thresholdedFrame);
        // Apply image processing techniques (e.g., erosion, dilation) to enhance detection
        // You can use Imgproc.erode and Imgproc.dilate functions for this.
        Imgproc.blur(hsvFrame, hsvFrame, new Size(15, 15));

        // Find and draw contours around white areas
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        //Imgproc.findContours(gFrame, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_TC89_L1);

        Imgproc.findContours(thresholdedFrame, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.drawContours(input, contours, -1, new Scalar(0, 0, 255), 7);
        Rect largestBBox = null;
        double largestArea = 0;

        for (MatOfPoint contour : contours) {
            Rect bbox = Imgproc.boundingRect(contour);
            if (bbox.area() > largestArea) {
                largestBBox = bbox;
            }
            if (bbox.area() > 3000) {
                positionx = bbox.x;
                positiony = bbox.y;
                Imgproc.rectangle(input, bbox, new Scalar(255, 0, 0), 2);
                Imgproc.arrowedLine(input, new Point(160, 120), new Point(bbox.x, bbox.y), new Scalar(0, 255, 0), 2);

            }
        }
        if (largestBBox != null) {
            positionx = largestBBox.x;
            positiony = largestBBox.y;
            Imgproc.rectangle(input, largestBBox, new Scalar(255, 0, 0), 2);
            Imgproc.arrowedLine(input, new Point(160, 120), new Point(largestBBox.x, largestBBox.y), new Scalar(0, 255, 0), 2);
        }

        area = largestArea;


        // Iterate through the contours and process them as needed
        //return gFrame;
        gFrame.release();
        hsvFrame.release();
        thresholdedFrame.release();


        return input;
        //return input;

    }
}
