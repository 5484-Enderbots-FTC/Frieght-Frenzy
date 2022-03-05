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

package org.firstinspires.ftc.teamcode.frieght_frenzy_code.auto;

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
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.ElementAnalysisPipelineFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.autoTrajectories;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.hardwareFF;
import org.firstinspires.ftc.teamcode.frieght_frenzy_code.var;

import java.util.ArrayList;

@Autonomous(name = "blue warehouse front", group = "blue")
public class AutoBlueWarehouseFrontW extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    double runningOpMode = 3;
    Pose2d intakeEnd;
    ElapsedTime intakeTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initWebcam();
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseBW);

        Trajectory toBlueHub3 = drive.trajectoryBuilder(traj.startPoseBW)
                .splineToConstantHeading(traj.blueHub3, Math.toRadians(180))
                .build();

        Trajectory toBlueHub2 = drive.trajectoryBuilder(traj.startPoseBW)
                .splineToConstantHeading(traj.blueHub2, Math.toRadians(180))
                .build();

        Trajectory toBlueHub1 = drive.trajectoryBuilder(traj.startPoseBW)
                .splineToConstantHeading(traj.blueHub1, Math.toRadians(180))
                .build();

        Trajectory toPark1_3 = drive.trajectoryBuilder(toBlueHub3.end(), true)
                .lineTo(traj.toParkBluePos1)
                .build();
        Trajectory toPark1_2 = drive.trajectoryBuilder(toBlueHub2.end(), true)
                .lineTo(traj.toParkBluePos1)
                .build();
        Trajectory toPark1_1 = drive.trajectoryBuilder(toBlueHub1.end(), true)
                .lineTo(traj.toParkBluePos1)
                .build();
        Trajectory toPark2 = drive.trajectoryBuilder(toPark1_3.end(), true)
                .lineTo(traj.toParkBluePos2)
                .build();

        Trajectory goCollect = drive.trajectoryBuilder(toPark2.end(), true)
                .forward(-50, FFMecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build(); //TODO: make sure -50 is right or if it needs to reverse

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
                    //telemetry.addLine(String.format("%s: Width=%f, Height=%f, Angle=%f", element.object.toString(), element.rectWidth, element.rectHeight, element.angle));
                    telemetry.addLine("Ratio of W/H: " + element.rectWidth / element.rectHeight);
                    telemetry.addLine("Section: " + element.section);
                    telemetry.addData("OpMode: ", runningOpMode);
                    if (element.section == ElementAnalysisPipelineFF.Section.LEFT) {
                        runningOpMode = 1;
                    } else if (element.section == ElementAnalysisPipelineFF.Section.MID) {
                        runningOpMode = 2;
                    } else if (element.section == ElementAnalysisPipelineFF.Section.RIGHT) {
                        runningOpMode = 3;
                    }

                }
            }
            telemetry.update();
        }

        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            /**
             * move arm and turret at same time
             */
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (runningOpMode == 3) {
                robot.movearm(var.armInitPower, var.thirdLvl);
            } else if (runningOpMode == 2) {
                robot.movearm(var.armInitPower, var.secondLvl);
            } else if (runningOpMode == 1) {
                robot.movearm(var.armInitPower, var.firstLvl);
            }
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!robot.midLimit.isPressed()) {
                telemetry.addData("pose estimate: ", drive.getPoseEstimate());
                telemetry.update();
                robot.mtrTurret.setPower(-0.4);
            }
            robot.mtrTurret.setPower(0);
            while (robot.mtrArm.isBusy()) {
                telemetry.addLine("weeeee arm finish");
                telemetry.addData("pose estimate: ", drive.getPoseEstimate());
            }
            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            /**
             * go to red hub and spit out bloque
             * then go to wall
             */
            if (runningOpMode == 3) {
                robot.svoIntakeTilt.setPosition(var.intakeHigh);
                drive.followTrajectory(toBlueHub3);
                spitOutBlock(false);
                drive.followTrajectory(toPark1_3);
            } else if (runningOpMode == 2) {
                robot.svoIntakeTilt.setPosition(var.intakeMid);
                drive.followTrajectory(toBlueHub2);
                spitOutBlock(false);
                drive.followTrajectory(toPark1_2);
            } else if (runningOpMode == 1) {
                robot.svoIntakeTilt.setPosition(var.intakeLow);
                drive.followTrajectory(toBlueHub1);
                spitOutBlock(false);
                drive.followTrajectory(toPark1_1);
            }
            robot.svoIntakeTilt.setPosition(var.intakeCollect);

            /**
             * set turret to go collect pos and arm go down
             */

            robot.mtrTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.mtrTurret.setPower(-0.3);
            telemetry.addLine("MOAR THINSG");
            telemetry.update();
            while (!robot.backLimit.isPressed() | !robot.bottomLimit.isPressed()) {
                telemetry.addLine("turret go brrrrr");

                if (robot.mtrTurret.getCurrentPosition() <= -900 && !robot.bottomLimit.isPressed()) {
                    robot.mtrArm.setPower(0.5);
                    telemetry.addLine("arm go brrrrrrrrrrrrrrrrrrrrrrrrrr");

                }
                if (robot.bottomLimit.isPressed()) {
                    robot.mtrArm.setPower(0);
                    telemetry.addLine("arm stoop");
                }
                if (robot.frontLimit.isPressed()) {
                    robot.mtrTurret.setPower(0);
                    telemetry.addLine("turret stoop");
                }
                telemetry.update();
            }
            robot.mtrTurret.setPower(0);
            robot.movearm(0.7, 150);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.mtrArm.isBusy()) {

            }
            robot.mtrArm.setPower(0);
            robot.svoIntakeTilt.setPosition(var.intakeCollect - 0.06);

            drive.followTrajectory(toPark2);
            /**
             * drive into warehouse for consumption
             */

            robot.svoIntake.setPower(var.lessPower * 1.5);
            drive.followTrajectoryAsync(goCollect);
            intakeTime.reset();
            while (robot.intakeLimit.isPressed() && intakeTime.seconds() <= var.intakeStopTime) {
                telemetry.addLine("consuming");
                telemetry.update();
                drive.update();
                drive.updatePoseEstimate();
            }
            if (intakeTime.time() < var.intakeStopTime) {
                drive.cancelFollowing();

                intakeEnd = drive.getPoseEstimate();
                drive.setDrivePower(new Pose2d());
                drive.update();
                telemetry.addLine("consumed");
                telemetry.addData("intake end: ", intakeEnd);
                telemetry.update();
                robot.svoIntake.setPower(0);

                /**
                 * has been consumed, now go to hub (and move arm/turret)
                 */

                //TODO: update later to be during trajectory on way to hub :)
                robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.movearm(var.armInitPower, var.thirdLvl);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.mtrArm.getCurrentPosition() >= -1000) {
                    telemetry.addData("haha", robot.mtrArm.getCurrentPosition());
                    telemetry.update();
                }
                while (!robot.midLimit.isPressed()) {
                    robot.mtrTurret.setPower(0.3);
                }
                robot.mtrTurret.setPower(0);
                while (robot.mtrArm.isBusy()) {
                    telemetry.addLine("weeeee arm finish");
                    telemetry.update();
                    //drive.update();
                }
                //drive.updatePoseEstimate();
                robot.mtrArm.setPower(0);
                robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                Trajectory goBack = drive.trajectoryBuilder(intakeEnd)
                        .splineToConstantHeading(traj.blueHub3, Math.toRadians(-90))
                        .build();

                drive.followTrajectory(goBack);

                spitOutBlock(true);

                /**
                 * send the turret back and arm down to collect again/ park
                 */
                drive.followTrajectory(toPark1_3);
                drive.followTrajectory(toPark2);
                robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (!robot.backLimit.isPressed()) {
                    robot.mtrTurret.setPower(-0.4);
                }
                robot.mtrTurret.setPower(0);
            }
            drive.cancelFollowing();
            break;
        }
    }

    public void spitOutBlock(boolean warehouse_block) {
        if (warehouse_block) {
            robot.svoIntakeTilt.setPosition(var.intakeHigh);
        } else if (runningOpMode == 3) {
            robot.svoIntakeTilt.setPosition(var.intakeHigh);
        } else if (runningOpMode == 2) {
            robot.svoIntakeTilt.setPosition(var.intakeMid);
        } else if (runningOpMode == 1) {
            robot.svoIntakeTilt.setPosition(var.intakeLow);
        }
        sleep(1000);
        robot.svoIntake.setPower(-var.lessPower);
        sleep(1500);
        robot.svoIntake.setPower(0);
    }
}
