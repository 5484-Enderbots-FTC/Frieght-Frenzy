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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

@Disabled
@Autonomous
public class AutoBlueWarehouse extends LinearOpMode
{
    hardwareFF robot = new hardwareFF();

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        robot.initWebcam();
        int alliance_element_location = 0;


        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        robot.svoIntakeTilt.setPosition(var.intakeInit);
        sleep(5000);
        while (!isStarted())
            {
                // Don't burn an insane amount of CPU cycles in this sample because
                // we're not doing anything else

                // Figure out which stones the pipeline detected, and print them to telemetry
                ArrayList<ElementAnalysisPipelineFF.AnalyzedElement> elements = robot.pipeline.getDetectedElements();
                sleep(250);

                if(elements.isEmpty())
                {
                    telemetry.addLine("No objects detected");
                }
                else
                {
                    for(ElementAnalysisPipelineFF.AnalyzedElement element : elements)
                    {
                        telemetry.addLine(String.format("%s: Width=%f, Height=%f, Angle=%f", element.object.toString(), element.rectWidth, element.rectHeight, element.angle));
                        telemetry.addLine("Ratio of W/H: " + element.rectWidth/element.rectHeight);
                        telemetry.addLine("Section: " + element.section);
                        if (element.section == ElementAnalysisPipelineFF.Section.LEFT){
                            alliance_element_location = 1;
                        }
                        else if (element.section == ElementAnalysisPipelineFF.Section.MID){
                            alliance_element_location = 2;
                        }
                        else if (element.section == ElementAnalysisPipelineFF.Section.RIGHT){
                            alliance_element_location = 3;
                        }

                    }
                }

            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){
            robot.deinit();
            telemetry.addData("Alliance Element Location: ", alliance_element_location);
            telemetry.update();
            robot.svoIntake.setPower(0);
            if (alliance_element_location == 1){
                robot.movearm(0.7,var.firstLvl);
                while (robot.mtrArm.isBusy()){

                }
            }
            if (alliance_element_location == 2){
                robot.movearm(0.7,var.secondLvl);
                while (robot.mtrArm.isBusy()){

                }
            }
            if (alliance_element_location == 3){
                robot.svoIntakeTilt.setPosition(var.intakeHigh);
                robot.movearm(0.7,var.thirdLvl);
                while (robot.mtrArm.isBusy()){

                }
            }
            robot.svoIntake.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.forward(-0.4,-800);
            robot.strafe(0.25,1200);
            robot.svoIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.svoIntake.setPower(var.lessPower);
            sleep(3000);
            robot.svoIntake.setPower(0);
            robot.strafe(-0.25,-1250);
            robot.forward(0.4,1800);
            robot.strafe(0.4,var.parkStrafe);
            robot.forward(0.4,-1*var.parkBack);
            break;
        }
    }
}