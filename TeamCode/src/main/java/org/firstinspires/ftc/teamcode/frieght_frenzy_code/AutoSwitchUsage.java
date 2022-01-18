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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple stones, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@Autonomous(name = "full auto", group = "auto")
@Disabled

public class AutoSwitchUsage extends LinearOpMode {
    hardwareFF robot = new hardwareFF();

    State currentState;

    double strafeSwapper = 1;

    public enum shippingHub {
        bottom,
        middle,
        top
    }

    public enum State {
        DUCK,
        DETECT_BARCODE,
        LEFT,
        MIDDLE,
        RIGHT,
        STOP
    }

    /***
     *
     * Auto steps:
     * 1. Detect Barcode to determine left middle or right
     * 2. Do carousel
     * 3. Split auto within left, middle, and right states into the in/out position
     *      - take into account alliance within this phase
     * 3. Place the preloaded block
     * 5. Park lol
     *
     */

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initWebcam();
        robot.mtrArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int alliance_element_location = 0;

        /**
         * Switches
         */

        if (robot.alliance == robot.red) {
            strafeSwapper = 1;
            telemetry.addLine("red alliance");
        }
        if (robot.alliance == robot.blue) {
            strafeSwapper = -1;
            telemetry.addLine("blue alliance");
        }

        if (robot.position == robot.carousel) {
            telemetry.addLine("carousel side");
        }
        if (robot.position == robot.warehouse) {
            telemetry.addLine("warehouse side");
        }
        telemetry.update();

        /**
         * Webcam things
         */

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);

        sleep(5000);
        while (!isStarted()) {
            // Don't burn an insane amount of CPU cycles in this sample because
            // we're not doing anything else

            // Figure out which stones the pipeline detected, and print them to telemetry
            ArrayList<ElementAnalysisPipelineFF.AnalyzedElement> elements = robot.pipeline.getDetectedElements();
            sleep(250);

            if (elements.isEmpty()) {
                telemetry.addLine("No objects detected");
            } else {
                for (ElementAnalysisPipelineFF.AnalyzedElement element : elements) {
                    telemetry.addLine(String.format("%s: Width=%f, Height=%f, Angle=%f", element.object.toString(), element.rectWidth, element.rectHeight, element.angle));
                    telemetry.addLine("Ratio of W/H: " + element.rectWidth / element.rectHeight);
                    telemetry.addLine("Section: " + element.section);
                    if (element.section == ElementAnalysisPipelineFF.Section.LEFT) {
                        alliance_element_location = 1;
                    } else if (element.section == ElementAnalysisPipelineFF.Section.MID) {
                        alliance_element_location = 2;
                    } else if (element.section == ElementAnalysisPipelineFF.Section.RIGHT) {
                        alliance_element_location = 3;
                    }

                }
            }

            telemetry.update();
        }

        /**
         *
         * Start of Code
         *
         */

        waitForStart();

        //arm movements to 'initialize' the arm happen outside the switch state cuz it's the same for all of em

        //move arm up slightly (encoders)

        //use the intake tilt servo to get it out in releasing position

        //turret spin to middle ish doesn't have to be perfect

        //lift the arm to bottom slot deposit height

        //THEN switch to DUCK
        currentState = State.DUCK;

        while (opModeIsActive()) {
            switch (currentState) {

                case DUCK:

                    if (robot.position == robot.carousel) {
                        telemetry.addLine("carousel side");
                        telemetry.update();
                        //drive to duck

                        //do duck

                        currentState = State.DETECT_BARCODE;
                    }
                    if (robot.position == robot.warehouse) {
                        telemetry.addLine("warehouse side");
                        telemetry.update();
                        //go around other robot then get to same position for duck

                        //do duck

                        currentState = State.DETECT_BARCODE;
                    }
                    break;

                case DETECT_BARCODE:
                    if (alliance_element_location == 1) {
                        currentState = State.LEFT;
                    } else if (alliance_element_location == 2) {
                        currentState = State.MIDDLE;
                    } else if (alliance_element_location == 3) {
                        currentState = State.RIGHT;
                    }
                    break;

                case LEFT:
                    telemetry.addLine("Barcode Position: Left");
                    telemetry.update();
                    //left and forward to bottom slot

                    //run intake backwards to spit out

                    //park

                    currentState = State.STOP;
                    break;

                case MIDDLE:
                    telemetry.addLine("Barcode Position: Middle");
                    telemetry.update();

                    currentState = State.STOP;
                    break;

                case RIGHT:
                    telemetry.addLine("Barcode Position: Right");
                    telemetry.update();

                    currentState = State.STOP;
                    break;

                case STOP:
                    break;
            }

            if (alliance_element_location == 1) {
                telemetry.addLine("Barcode Position: Left");
            } else if (alliance_element_location == 2) {
                telemetry.addLine("Barcode Position: Middle");
            } else if (alliance_element_location == 3) {
                telemetry.addLine("Barcode Position: Right");
            }

            telemetry.update();
            //robot.forward(0.2,300);
            //robot.strafe(0.2,100);
            //robot.forward(0.3,100);
            //robot.carouselServo.setPosition(1);
            //robot.turn(0.2,100);
        }
    }
}