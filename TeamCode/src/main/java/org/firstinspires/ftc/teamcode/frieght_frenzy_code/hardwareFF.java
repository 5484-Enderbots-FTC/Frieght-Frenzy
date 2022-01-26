package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import android.media.AudioManager;
import android.media.SoundPool;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.R;
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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;

public class hardwareFF {
    //common hardware imports are stored here so that its crazy easy to import them and make changes to ALL CODE
    //documents at the same time instead of doing it manually

    /**

    * the variables here that are only used in this doc can be private but everything else like motors and servos need to be
    * public so they can be called in the other programs.

    * the methods and classes are public so you can call them and have them reference the other things in here

    //So far this season we just have motors, so I've done the work to initialize them here:
     */
    // the code above is what goes in the init(), and to play the sound use mySound.play(honkID,1,1,1,0,1);
    public DcMotorEx mtrBL, mtrBR, mtrFL, mtrFR, mtrNEHL;

    private HardwareMap hw = null;



        //common hardware imports are stored here so that its crazy easy to import them and make changes to ALL CODE
        //documents at the same time instead of doing it manually

        /**
         * the variables here that are only used in this doc can be private but everything else like motors and servos need to be
         * public so they can be called in the other programs.
         * <p>
         * the methods and classes are public so you can call them and have them reference the other things in here
         * <p>
         * //So far this season we just have motors, so I've done the work to initialize them here:
         */
        public DcMotorEx mtrArm, mtrTurret;
        public CRServo svoCarousel, svoIntake; //servo port 0, 1
        public Servo svoIntakeTilt, LEDstrip;
        public DigitalChannel alliance_switch, position_switch;;
        public VoltageSensor batteryVoltage;
        public TouchSensor leftLimit, rightLimit, topLimit, bottomLimit;
        public double alliance = 0;
        public final double red = 1;
        public final double blue = -1;

        public double position =0;
        public final double carousel=0;
        public final double warehouse=-1;

        ElapsedTime definitWait = new ElapsedTime();
        public hardwareFF() {
            //nothing goes in this- its just a way to call the program
        }

        public void init(HardwareMap thisHwMap) {
            hw = thisHwMap;

            mtrBL = hw.get(DcMotorEx.class, "mtrBL");
            mtrBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            mtrBL.setDirection(DcMotorEx.Direction.FORWARD);

            mtrBR = hw.get(DcMotorEx.class, "mtrBR");
            mtrBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            mtrBR.setDirection(DcMotorEx.Direction.REVERSE);

            mtrFL = hw.get(DcMotorEx.class, "mtrFL");
            mtrFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            mtrFL.setDirection(DcMotorEx.Direction.FORWARD);

            mtrFR = hw.get(DcMotorEx.class, "mtrFR");
            mtrFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            mtrFR.setDirection(DcMotorEx.Direction.REVERSE);

            mtrArm = hw.get(DcMotorEx.class, "mtrArm");
            mtrArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            mtrArm.setDirection(DcMotorEx.Direction.FORWARD);

            mtrTurret = hw.get(DcMotorEx.class, "mtrTurret");
            mtrTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            mtrTurret.setDirection(DcMotorEx.Direction.FORWARD);

            svoCarousel = hw.get(CRServo.class, "svoCarousel");
            svoCarousel.setDirection(CRServo.Direction.REVERSE);

            svoIntake = hw.get(CRServo.class, "svoIntake");
            svoIntake.setDirection(CRServo.Direction.FORWARD);

            svoIntakeTilt = hw.get(Servo.class, "svoIntakeTilt");
            LEDstrip = hw.get(Servo.class,"LEDstrip");

            leftLimit = hw.get(TouchSensor.class, "leftLimit");
            rightLimit = hw.get(TouchSensor.class, "rightLimit");
            topLimit = hw.get(TouchSensor.class, "topLimit");
            bottomLimit = hw.get(TouchSensor.class, "bottomLimit");

            alliance_switch = hw.get(DigitalChannel.class, "alliance_switch");
            position_switch = hw.get(DigitalChannel.class, "position_switch");

            batteryVoltage = hw.voltageSensor.iterator().next();

            LEDstrip.setPosition(var.rainbowo);
            if (alliance_switch.getState() == true) {
                alliance = red;
            }
            if (alliance_switch.getState() == false) {
                alliance = blue;
            }
            if (position_switch.getState() == true) {
                position = red;
            }
            if (position_switch.getState() == false) {
                position = blue;
            }

        }

        public void initWebcam() {

        }

        public void initPID() {

        }


    public void updateDrive(double fwdStick, double turnStick, double strStick){
        mtrBL.setPower((fwdStick - turnStick + strStick));
        mtrBR.setPower((fwdStick + turnStick - strStick));
        mtrFL.setPower((fwdStick - turnStick - strStick));
        mtrFR.setPower((fwdStick + turnStick + strStick));
    }

    public void updateDrive(double fwdStick, double turnStick, double strStick, boolean reversed){
        mtrBL.setPower((-fwdStick + turnStick - strStick));
        mtrBR.setPower((-fwdStick - turnStick + strStick));
        mtrFL.setPower((-fwdStick + turnStick + strStick));
        mtrFR.setPower((-fwdStick - turnStick - strStick));
    }
    public void brake (){
            mtrFL.setPower(0);
            mtrFR.setPower(0);
            mtrBL.setPower(0);
            mtrBR.setPower(0);
    }
    public void reset() {
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void forward (double power, int position){
        reset();
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position*adjustment);
        mtrBL.setPower(power);
        mtrBL.setTargetPosition(position*adjustment);
        mtrFR.setPower(power);
        mtrFR.setTargetPosition(position*adjustment);
        mtrFL.setPower(power);
        mtrFL.setTargetPosition(position*adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(mtrFR.isBusy()){

        }
        brake();
    }
    public void strafe (double power, int position){
        reset();
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position*adjustment);
        mtrBL.setPower(-power);
        mtrBL.setTargetPosition(-position*adjustment);
        mtrFR.setPower(-power);
        mtrFR.setTargetPosition(-position*adjustment);
        mtrFL.setPower(power);
        mtrFL.setTargetPosition(position*adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(mtrFR.isBusy()){

        }
        brake();
    }
    public void turn (double power, int position){
        reset();
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position*adjustment);
        mtrBL.setPower(-power);
        mtrBL.setTargetPosition(position*adjustment);
        mtrFR.setPower(power);
        mtrFR.setTargetPosition(position*adjustment);
        mtrFL.setPower(-power);
        mtrFL.setTargetPosition(position*adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(mtrFR.isBusy()){

        }

        brake();
    }
    public void movearm (double power, int position){
        int adjustment = 1;
        mtrArm.setPower(-power);
        mtrArm.setTargetPosition(-position*adjustment);
        //mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void deinit(){
        mtrArm.setPower(-0.7);
        mtrArm.setTargetPosition(-var.deinitArm);
        mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (mtrArm.isBusy()){

        }
        mtrArm.setPower(0);
        definitWait.reset();
        svoIntakeTilt.setPosition(var.intakeTiltMid);
        while(definitWait.seconds()<0.25){

        }
        svoIntakeTilt.setPosition(var.intakeTiltCollect);


    }
    public void align (double power, int position) {
        reset();
        //do math, make things do things, define the adjustment function via wacky programming
        int adjustment = 1;
        mtrBR.setPower(power);
        mtrBR.setTargetPosition(position * adjustment);
        mtrBL.setPower(power);
        mtrBL.setTargetPosition(position * adjustment);
        mtrFR.setPower(power);
        mtrFR.setTargetPosition(position * adjustment);
        mtrFL.setPower(power);
        mtrFL.setTargetPosition(position * adjustment);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (mtrFR.isBusy()) {
        }
        brake();
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
            voltroned_code.ElementAnalysisPipeline.ObjectType object;
            double angle;
            double rectWidth;
            double rectHeight;
            double WidthHeightRatio;
            voltroned_code.ElementAnalysisPipeline.Section section;

        }
        enum Section
        {
            LEFT,
            MID,
            RIGHT
        }
        enum ObjectType
        {
            PWR_SHOT,
            RED_GOAL
        }

        ArrayList<voltroned_code.ElementAnalysisPipeline.AnalyzedElement> internalElementList = new ArrayList<>();
        volatile ArrayList<voltroned_code.ElementAnalysisPipeline.AnalyzedElement> clientElementList = new ArrayList<>();

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

        voltroned_code.ElementAnalysisPipeline.Stage[] stages = voltroned_code.ElementAnalysisPipeline.Stage.values();

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

        public ArrayList<voltroned_code.ElementAnalysisPipeline.AnalyzedElement> getDetectedElements()
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
                voltroned_code.ElementAnalysisPipeline.ContourRegionAnalysis aboveMidlineMetrics = analyzeContourRegion(aboveMidline);
                voltroned_code.ElementAnalysisPipeline.ContourRegionAnalysis belowMidlineMetrics = analyzeContourRegion(belowMidline);

                if (aboveMidlineMetrics == null || belowMidlineMetrics == null) {
                    return; // Get out of dodge
                }

                //Point displOfOrientationLinePoint2 = computeDisplacementForSecondPointOfStoneOrientationLine(rotatedRectFitToContour, rotRectAngle);

                voltroned_code.ElementAnalysisPipeline.AnalyzedElement analyzedElement = new voltroned_code.ElementAnalysisPipeline.AnalyzedElement();
                analyzedElement.angle = rotRectAngle;
                analyzedElement.WidthHeightRatio = rotatedRectFitToContour.size.width / rotatedRectFitToContour.size.height;
                analyzedElement.object = voltroned_code.ElementAnalysisPipeline.ObjectType.RED_GOAL;
                drawTagText(rotatedRectFitToContour, "Blue Capstone Thing", input);
                analyzedElement.rectWidth = rotatedRectFitToContour.size.width;
                analyzedElement.rectHeight = rotatedRectFitToContour.size.height;
                if (rotatedRectFitToContour.center.x <= DIVISION_ONE) {
                    analyzedElement.section = voltroned_code.ElementAnalysisPipeline.Section.LEFT;
                } else if (rotatedRectFitToContour.center.x > DIVISION_ONE && rotatedRectFitToContour.center.x < DIVISION_TWO) {
                    analyzedElement.section = voltroned_code.ElementAnalysisPipeline.Section.MID;
                } else if (rotatedRectFitToContour.center.x >= DIVISION_TWO) {
                    analyzedElement.section = voltroned_code.ElementAnalysisPipeline.Section.RIGHT;
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

        static voltroned_code.ElementAnalysisPipeline.ContourRegionAnalysis analyzeContourRegion(ArrayList<Point> contourPoints)
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

                voltroned_code.ElementAnalysisPipeline.ContourRegionAnalysis analysis = new voltroned_code.ElementAnalysisPipeline.ContourRegionAnalysis();
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



