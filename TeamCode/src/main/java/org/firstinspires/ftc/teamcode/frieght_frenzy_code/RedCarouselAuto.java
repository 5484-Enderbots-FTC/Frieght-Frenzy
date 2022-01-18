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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class RedCarouselAuto extends LinearOpMode
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

                //what did u detect
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
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.deinit();
            telemetry.addData("雪花飘飘北风啸啸 Alliance Element Location: ", alliance_element_location);
            telemetry.update();
            robot.strafe(0.5,450);
            //robot.forward(0.4,1000);

            robot.strafe(0.5,450);
            robot.forward(0.4,1000);
            //robot.strafe(-0.2,-125);

            robot.mtrBR.setPower(-0.4);
            robot.mtrBL.setPower(0.4);
            robot.mtrFR.setPower(0.4);
            robot.mtrFL.setPower(-0.4);
            sleep(2000);
            robot.brake();

            //robot.strafe(-0.060,-100);
            /*
            robot.mtrBR.setPower(-0.1);
            robot.mtrBL.setPower(0.1);
            robot.mtrFR.setPower(0.1);
            robot.mtrFL.setPower(-0.1);
            sleep(2000);
            robot.brake();
`           */
            robot.mtrBR.setPower(0.3);
            robot.mtrBL.setPower(0.3);
            robot.mtrFR.setPower(0.3);
            robot.mtrFL.setPower(0.3);
            sleep(500);
            robot.brake();
            //use distance sensor here instead of power strafe
            robot.svoCarousel.setPower(1);
            sleep(3000);
            robot.svoCarousel.setPower(0);

            if (alliance_element_location == 1){
                robot.movearm(0.7,var.firstLvl);
                robot.forward(-0.4,-1600);
                while (robot.mtrArm.isBusy()){

                }
            }
            if (alliance_element_location == 2){
                robot.movearm(0.7,var.secondLvl);
                robot.forward(-0.4,-1700);
                while (robot.mtrArm.isBusy()){

                }
            }
            if (alliance_element_location == 3){

                //robot.svoIntakeTilt.setPosition(var.intakeTiltMid);
                robot.svoIntakeTilt.setPosition(var.intakeTiltHigh);
                robot.forward(-0.4,-1900);
                robot.movearm(0.7,var.thirdLvl);
                while (robot.mtrArm.isBusy()){

                }
            }
            sleep(500);
            robot.strafe(0.25,600);
            //use distance sensor here instead of strafe
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.svoIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.svoIntake.setPower(var.lessPower);
            sleep(3000);
            robot.svoIntake.setPower(0);
            robot.strafe(-0.25,-600);
            robot.forward(0.5,2200);

            robot.mtrBR.setPower(-0.4);
            robot.mtrBL.setPower(0.4);
            robot.mtrFR.setPower(0.4);
            robot.mtrFL.setPower(-0.4);
            sleep(2000);
            robot.brake();


            //robot.strafe(-0.060,-100);

            /*robot.mtrBR.setPower(-0.1);
            robot.mtrBL.setPower(0.1);
            robot.mtrFR.setPower(0.1);
            robot.mtrFL.setPower(-0.1);
            sleep(2000);
            robot.brake();
            */

            robot.mtrBR.setPower(0.3);
            robot.mtrBL.setPower(0.3);
            robot.mtrFR.setPower(0.3);
            robot.mtrFL.setPower(0.3);
            sleep(500);
            robot.brake();

            robot.movearm(0.7,var.thirdLvl);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.strafe(0.3,1075);
            //use distance sensor here instead of the strafe
            break;

        }


    }
}