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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;

import java.util.ArrayList;

@Autonomous(name = "autoRedCarousel w/ webcam")
public class AutoRedCarouslWithWeb extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    double runningOpMode = 3;
    Pose2d endDepPos;

    @Override
    public void runOpMode() {
        //this will init EVERYTHING on the robot
        robot.init(hardwareMap);
        robot.initWebcam();
        //FFMecanum must be called AFTER the robot init bc just the motors need to be overridden.
        FFMecanumDrive drive = new FFMecanumDrive(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(new Vector2d(-63, -58), Math.toRadians(180))
                .addTemporalMarker(0.9,0, ()->{
                    robot.svoCarousel.setPower(1);
                })
                .build();

        Trajectory toRedHub3 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeHigh);
                })
                .build();

        Trajectory toRedHub2 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -52), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeMid);
                })
                .build();

        Trajectory toRedHub1 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    robot.svoIntakeTilt.setPosition(var.intakeLow);
                })
                .build();

        Trajectory toPark1 = drive.trajectoryBuilder(endDepPos)
                .lineTo(traj.toParkPos1)
                .build();
        Trajectory toPark2 = drive.trajectoryBuilder(toPark1.end())
                .lineTo(traj.toParkPos2)
                .build();

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        robot.svoIntakeTilt.setPosition(var.intakeInit);
        sleep(4000);
        while (!isStarted()) {
            //what did u detect
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
                        runningOpMode = 1;
                    } else if (element.section == ElementAnalysisPipelineFF.Section.MID) {
                        runningOpMode = 2;
                    } else if (element.section == ElementAnalysisPipelineFF.Section.RIGHT) {
                        runningOpMode = 3;
                    }

                }
            }
        }

        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            /*
                robot.movearm(0.7,var.thirdLvl);
                while (robot.mtrArm.isBusy()){
                }
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.svoCarousel.setPower(1);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             */
            drive.followTrajectory(toRedCarousel);
            sleep(2500);
            robot.svoCarousel.setPower(0);
            if (runningOpMode == 3) {
                drive.followTrajectory(toRedHub3);
                endDepPos = toRedHub3.end();
            } else if (runningOpMode == 2) {
                drive.followTrajectory(toRedHub2);
                endDepPos = toRedHub2.end();
            } else if (runningOpMode == 1) {
                drive.followTrajectory(toRedHub1);
                endDepPos = toRedHub1.end();
            }
            robot.svoIntake.setPower(-var.lessPower);
            sleep(1500);
            robot.svoIntake.setPower(0);
            if (runningOpMode == 1) {
                robot.svoIntakeTilt.setPosition(0.9);
            }
            drive.followTrajectory(toPark1);
            drive.followTrajectory(toPark2);
            break;
        }
    }
}