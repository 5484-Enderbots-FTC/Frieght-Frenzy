/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.var;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple stones, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@Autonomous
public class RochesterAuto extends LinearOpMode
{
    OpenCvWebcam webcam;
    ElementAnalysisPipeline pipeline;
    hardwareFF robot = new hardwareFF();

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera2Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */
        int alliance_element_location = 0;
        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        // Open async and start streaming inside opened callback
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                pipeline = new ElementAnalysisPipeline();
                webcam.setPipeline(pipeline);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        robot.svoIntakeTilt.setPosition(var.intakeInit);
        sleep(5000);
        while (!isStarted())
            {
                // Don't burn an insane amount of CPU cycles in this sample because
                // we're not doing anything else

                // Figure out which stones the pipeline detected, and print them to telemetry
                ArrayList<ElementAnalysisPipeline.AnalyzedElement> elements = pipeline.getDetectedElements();
                sleep(250);

                if(elements.isEmpty())
                {
                    telemetry.addLine("No objects detected");
                }
                else
                {
                    for(ElementAnalysisPipeline.AnalyzedElement element : elements)
                    {
                        telemetry.addLine(String.format("%s: Width=%f, Height=%f, Angle=%f", element.object.toString(), element.rectWidth, element.rectHeight, element.angle));
                        telemetry.addLine("Ratio of W/H: " + element.rectWidth/element.rectHeight);
                        telemetry.addLine("Section: " + element.section);
                        if (element.section == ElementAnalysisPipeline.Section.LEFT){
                            alliance_element_location = 1;
                        }
                        else if (element.section == ElementAnalysisPipeline.Section.MID){
                            alliance_element_location = 2;
                        }
                        else if (element.section == ElementAnalysisPipeline.Section.RIGHT){
                            alliance_element_location = 3;
                        }

                    }
                }

            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){
            //arm go brr
            //robot.svoIntakeTilt.setPosition(var.intakeTiltMid);
            //robot.svoIntakeTilt.setPosition(var.intakeTiltCollect);
            telemetry.addData("雪花飘飘北风啸啸 Alliance Element Location: ", alliance_element_location);
            telemetry.update();
            robot.strafe(0.5,350);
            robot.forward(0.4,1000);
            robot.forward(0.3,500);
            robot.strafe(-0.2,-200);
            robot.svoCarousel.setPower(1);
            sleep(3000);
            robot.svoCarousel.setPower(0);
            //robot.turn(0.2,100);
            break;
        }


    }

    static class ElementAnalysisPipeline extends OpenCvPipeline
    {
        /*
         * Our working image buffers
         */
        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        /*
         * Threshold values
         */
        static final int CB_CHAN_MASK_THRESHOLD = 165;
        static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;

        /*
         * The elements we use for noise reduction
         */
        //erodeElement makes stuff smoller, dilateElement makes the current things more thicc
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 1));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));

        /*
         * Colors
         */
        static final Scalar TEAL = new Scalar(3, 148, 252);
        static final Scalar PURPLE = new Scalar(158, 52, 235);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final int CONTOUR_LINE_THICKNESS = 2;
        static final int CB_CHAN_IDX = 2;
        static final int CR_CHAN_IDX = 1;

        static final int DIVISION_ONE = 90;
        static final int DIVISION_TWO = 200;

        static class AnalyzedElement
        {
            ObjectType object;
            double angle;
            double rectWidth;
            double rectHeight;
            double WidthHeightRatio;
            Section section;

        }
        enum Section
        {
            LEFT,
            MID,
            RIGHT
        }
        enum ObjectType
        {

            ALLIANCE_SHIPPING_ELEMENT
        }

        ArrayList<AnalyzedElement> internalElementList = new ArrayList<>();
        volatile ArrayList<AnalyzedElement> clientElementList = new ArrayList<>();

        /*
         * Some stuff to handle returning our various buffers
         */
        enum Stage
        {
            FINAL,
            Cb,
            MASK,
            MASK_NR,
            CONTOURS;
        }

        Stage[] stages = Stage.values();

        // Keep track of what stage the viewport is showing
        int stageNum = 0;

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int nextStageNum = stageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageNum = nextStageNum;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            // We'll be updating this with new data below
            internalElementList.clear();

            /*
             * Run the image processing
             */
            for(MatOfPoint contour : findContours(input))
            {
                analyzeContour(contour, input);
            }

            clientElementList = new ArrayList<>(internalElementList);

            /*
             * Decide which buffer to send to the viewport
             */
            switch (stages[stageNum])
            {
                case Cb:
                {
                    return cbMat;
                }

                case FINAL:
                {
                    return input;
                }

                case MASK:
                {
                    return thresholdMat;
                }

                case MASK_NR:
                {
                    return morphedThreshold;
                }

                case CONTOURS:
                {
                    return contoursOnPlainImageMat;
                }
            }

            return input;
        }

        public ArrayList<AnalyzedElement> getDetectedElements()
        {
            return clientElementList;
        }

        ArrayList<MatOfPoint> findContours(Mat input)
        {
            // A list we'll be using to store the contours we find
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);

            // Threshold the Cb channel to form a mask, then run some noise reduction
            Imgproc.threshold(cbMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
            morphMask(thresholdMat, morphedThreshold);

            // Ok, now actually look for the contours! We only look for external contours.
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            // We do draw the contours we find, but not to the main input buffer.
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

            return contoursList;
        }

        void morphMask(Mat input, Mat output)
        {
            /*
             * Apply some erosion and dilation for noise reduction
             */

            //Imgproc.erode(input, output, erodeElement);
            //Imgproc.erode(output, output, erodeElement);

            Imgproc.dilate(input, output, dilateElement);
            //Imgproc.dilate(output, output, dilateElement);
        }

        void analyzeContour(MatOfPoint contour, Mat input)
        {
            // Transform the contour to a different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

            if (rotatedRectFitToContour.size.width > 60 && rotatedRectFitToContour.size.height > 30) {
                drawRotatedRect(rotatedRectFitToContour, input);

                // The angle OpenCV gives us can be ambiguous, so look at the shape of
                // the rectangle to fix that.
                double rotRectAngle = rotatedRectFitToContour.angle;
                if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
                    rotRectAngle += 90;
                }

                // Figure out the slope of a line which would run through the middle, lengthwise
                // (Slope as in m from 'Y = mx + b')
                double midlineSlope = Math.tan(Math.toRadians(rotRectAngle));

                // We're going to split the this contour into two regions: one region for the points
                // which fall above the midline, and one region for the points which fall below.
                // We'll need a place to store the points as we split them, so we make ArrayLists
                ArrayList<Point> aboveMidline = new ArrayList<>(points.length / 2);
                ArrayList<Point> belowMidline = new ArrayList<>(points.length / 2);

                // Ok, now actually split the contour into those two regions we discussed earlier!
                for (Point p : points) {
                    if (rotatedRectFitToContour.center.y - p.y > midlineSlope * (rotatedRectFitToContour.center.x - p.x)) {
                        aboveMidline.add(p);
                    } else {
                        belowMidline.add(p);
                    }
                }

                // Now that we've split the contour into those two regions, we analyze each
                // region independently.
                ContourRegionAnalysis aboveMidlineMetrics = analyzeContourRegion(aboveMidline);
                ContourRegionAnalysis belowMidlineMetrics = analyzeContourRegion(belowMidline);

                if (aboveMidlineMetrics == null || belowMidlineMetrics == null) {
                    return; // Get out of dodge
                }

                //Point displOfOrientationLinePoint2 = computeDisplacementForSecondPointOfStoneOrientationLine(rotatedRectFitToContour, rotRectAngle);

                AnalyzedElement analyzedElement = new AnalyzedElement();
                analyzedElement.angle = rotRectAngle;
                analyzedElement.WidthHeightRatio = rotatedRectFitToContour.size.width / rotatedRectFitToContour.size.height;
                analyzedElement.object = ObjectType.ALLIANCE_SHIPPING_ELEMENT;
                drawTagText(rotatedRectFitToContour, "Blue Capstone Thing", input);
                analyzedElement.rectWidth = rotatedRectFitToContour.size.width;
                analyzedElement.rectHeight = rotatedRectFitToContour.size.height;
                if (rotatedRectFitToContour.center.x <= DIVISION_ONE) {
                    analyzedElement.section = Section.LEFT;
                } else if (rotatedRectFitToContour.center.x > DIVISION_ONE && rotatedRectFitToContour.center.x < DIVISION_TWO) {
                    analyzedElement.section = Section.MID;
                } else if (rotatedRectFitToContour.center.x >= DIVISION_TWO) {
                    analyzedElement.section = Section.RIGHT;
                }
                internalElementList.add(analyzedElement);

            }
        }



        static class ContourRegionAnalysis
        {
            /*
             * This class holds the results of analyzeContourRegion()
             */

            double hullArea;
            double contourArea;
            double density;
            List<MatOfPoint> listHolderOfMatOfPoint;
        }

        static ContourRegionAnalysis analyzeContourRegion(ArrayList<Point> contourPoints)
        {
            // drawContours() requires a LIST of contours (there's no singular drawContour()
            // method), so we have to make a list, even though we're only going to use a single
            // position in it...
            MatOfPoint matOfPoint = new MatOfPoint();
            matOfPoint.fromList(contourPoints);
            List<MatOfPoint> listHolderOfMatOfPoint = Arrays.asList(matOfPoint);

            // Compute the convex hull of the contour
            MatOfInt hullMatOfInt = new MatOfInt();
            Imgproc.convexHull(matOfPoint, hullMatOfInt);

            // Was the convex hull calculation successful?
            if(hullMatOfInt.toArray().length > 0)
            {
                // The convex hull calculation tells us the INDEX of the points which
                // which were passed in eariler which form the convex hull. That's all
                // well and good, but now we need filter out that original list to find
                // the actual POINTS which form the convex hull
                Point[] hullPoints = new Point[hullMatOfInt.rows()];
                List<Integer> hullContourIdxList = hullMatOfInt.toList();

                for (int i = 0; i < hullContourIdxList.size(); i++)
                {
                    hullPoints[i] = contourPoints.get(hullContourIdxList.get(i));
                }

                ContourRegionAnalysis analysis = new ContourRegionAnalysis();
                analysis.listHolderOfMatOfPoint = listHolderOfMatOfPoint;

                // Compute the hull area
                analysis.hullArea = Imgproc.contourArea(new MatOfPoint(hullPoints));

                // Compute the original contour area
                analysis.contourArea = Imgproc.contourArea(listHolderOfMatOfPoint.get(0));

                // Compute the contour density. This is the ratio of the contour area to the
                // area of the convex hull formed by the contour
                analysis.density = analysis.contourArea / analysis.hullArea;

                return analysis;
            }
            else
            {
                return null;
            }
        }

        static Point computeDisplacementForSecondPointOfStoneOrientationLine(RotatedRect rect, double unambiguousAngle)
        {
            // Note: we return a point, but really it's not a point in space, we're
            // simply using it to hold X & Y displacement values from the middle point
            // of the bounding rect.
            Point point = new Point();

            // Figure out the length of the short side of the rect
            double shortSideLen = Math.min(rect.size.width, rect.size.height);

            // We draw a line that's 3/4 of the length of the short side of the rect
            double lineLength = shortSideLen * .75;

            // The line is to be drawn at 90 deg relative to the midline running through
            // the rect lengthwise
            point.x = (int) (lineLength * Math.cos(Math.toRadians(unambiguousAngle+90)));
            point.y = (int) (lineLength * Math.sin(Math.toRadians(unambiguousAngle+90)));

            return point;
        }

        static void drawTagText(RotatedRect rect, String text, Mat mat)
        {
            Imgproc.putText(
                    mat, // The buffer we're drawing on
                    text, // The text we're drawing
                    new Point( // The anchor point for the text
                            rect.center.x,  // x anchor point
                            rect.center.y+25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    0.6, // Font size
                    TEAL, // Font color
                    1); // Font thickness
        }

        static void drawRotatedRect(RotatedRect rect, Mat drawOn)
        {
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */

            Point[] points = new Point[4];
            rect.points(points);

            for(int i = 0; i < 4; ++i)
            {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
            }
        }
    }
}