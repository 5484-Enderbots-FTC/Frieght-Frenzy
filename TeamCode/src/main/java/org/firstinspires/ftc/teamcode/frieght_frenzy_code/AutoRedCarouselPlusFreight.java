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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.FFMecanumDriveCancelable;

import java.util.ArrayList;

/***
 * IF ALL THE MOVEMENT BREAKS ITS PROLLY BECAUSE OF THE NEW DRIVE W/ CANCELLATION
 */

@Autonomous(name = "autoRedCarousel w/ freight")
public class AutoRedCarouselPlusFreight extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    double runningOpMode = 3;
    Pose2d endDepPos;
    Pose2d intakeEnd;
    ElapsedTime intakeDistance = new ElapsedTime();
    double distanceTravelled = 0;

    private enum intakeMove {
        NOTHING,
        NONE,
        TURRETPOS,
        ARMPOWER,
        ARMPOS,
        COLLECTPATH

    }

    intakeMove currentIntakeState = intakeMove.NOTHING;

    @Override
    public void runOpMode() {
        //this will init EVERYTHING on the robot
        robot.init(hardwareMap);
        robot.initWebcam();
        //FFMecanum must be called AFTER the robot init bc just the motors need to be overridden.
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);

        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(new Vector2d(-63, -58), Math.toRadians(180))
                .addTemporalMarker(0.9, 0, () -> {
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
                .addTemporalMarker(0, () -> {
                    robot.mtrTurret.setPower(0.3);
                })
                .build();
        Trajectory toPark2 = drive.trajectoryBuilder(toPark1.end())
                .lineTo(traj.toParkPos2)
                .build();
        Trajectory intakeForward = drive.trajectoryBuilder(toPark2.end())
                .forward(10, FFMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0, () -> {
                    robot.svoIntake.setPower(var.intakeCollect);
                })
                .build();
        //TODO: add state machine for rotating the arm back (also actually rotating the arm in the first place as well)
        Trajectory backToRedHub3 = drive.trajectoryBuilder(intakeEnd)
                .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                .build();

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        robot.svoIntakeTilt.setPosition(var.intakeInit);
        sleep(5000);
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
            //might need to rearrange the followTrajectory statements cuz idk how they work with state machines
            //if the followTrajectory commands need to run the whole way, try putting the currentIntakeState changes in the
            //followTrajectory command as a marker maybe? or try to somehow make most of this a marker
            drive.followTrajectoryAsync(toPark1);
            currentIntakeState = intakeMove.TURRETPOS;
            switch (currentIntakeState) {
                case NOTHING:
                    break;
                case TURRETPOS:
                    if (drive.isBusy()) {
                        if (robot.rightLimit.isPressed()) {
                            robot.mtrTurret.setPower(0);
                        }
                    } else {
                        drive.followTrajectoryAsync(toPark2);
                        currentIntakeState = intakeMove.ARMPOWER;
                    }
                    break;
                case ARMPOWER:
                    //TODO: Add arm movements here and other places where it's needed
                    robot.mtrArm.setPower(-0.6);
                    currentIntakeState = intakeMove.ARMPOS;
                    break;
                case ARMPOS:
                    if(robot.bottomLimit.isPressed()){
                        robot.mtrArm.setPower(0);
                        robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        currentIntakeState = intakeMove.COLLECTPATH;
                    }else{
                    }
                    break;
                case COLLECTPATH:
                    if(!drive.isBusy()){
                        drive.followTrajectory(intakeForward);
                        currentIntakeState = intakeMove.NOTHING;
                    }
                    break;
            }
            //this might be totally wrong
            intakeDistance.reset();
            while (!robot.intakeLimit.isPressed()) {
                if (robot.intakeLimit.isPressed()) {
                    robot.svoIntake.setPower(0);
                    distanceTravelled = 10 * intakeDistance.time();
                    intakeEnd = new Pose2d(40, -67 + distanceTravelled);
                    drive.cancelFollowing();
                    break;
                }
                drive.update();
            }
            break;
        }
    }
}