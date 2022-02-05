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

@Autonomous(name = "full auto with optimize (in progress)")
public class FullAutoWithOptimize extends LinearOpMode {
    hardwareFF robot = new hardwareFF();
    autoTrajectories traj = new autoTrajectories();

    double runningOpMode;
    Pose2d intakeEnd;
    ElapsedTime duckTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initWebcam();
        FFMecanumDriveCancelable drive = new FFMecanumDriveCancelable(hardwareMap);

        drive.setPoseEstimate(traj.startPoseRC);


        Trajectory toRedCarousel = drive.trajectoryBuilder(traj.startPoseRC, true)
                .splineToConstantHeading(new Vector2d(-63, -58), Math.toRadians(180))
                /*
                .addTemporalMarker(0.9, 0, () -> {
                    robot.svoCarousel.setPower(1);
                    duckTimer.reset();
                })

                 */
                .build();

        Trajectory toRedHub3 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -47), Math.toRadians(0))
                .build();

        Trajectory toRedHub2 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -52), Math.toRadians(0))
                .build();

        Trajectory toRedHub1 = drive.trajectoryBuilder(toRedCarousel.end())
                .splineTo(new Vector2d(-12, -50), Math.toRadians(0))
                .build();

        Trajectory toWarehouse1_3 = drive.trajectoryBuilder(traj.redHub3Exit)
                .splineToConstantHeading(traj.toParkPos2,Math.toRadians(0))
                .build();

        Trajectory toWarehouse1_2 = drive.trajectoryBuilder(traj.redHub2Exit)
                .splineToConstantHeading(traj.toParkPos2,Math.toRadians(0))
                .build();

        Trajectory toWarehouse1_1 = drive.trajectoryBuilder(traj.redHub1Exit)
                .splineToConstantHeading(traj.toParkPos2,Math.toRadians(0))
                .build();

        Trajectory toPark1_3 = drive.trajectoryBuilder(toRedHub3.end())
                .lineTo(traj.toParkPos1)
                .build();

        Trajectory toPark1_2 = drive.trajectoryBuilder(toRedHub2.end())
                .lineTo(traj.toParkPos1)
                .build();

        Trajectory toPark1_1 = drive.trajectoryBuilder(toRedHub1.end())
                .lineTo(traj.toParkPos1)
                .build();

        Trajectory toPark2 = drive.trajectoryBuilder(toPark1_3.end())
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
            drive.followTrajectory(toRedCarousel);
            robot.svoCarousel.setPower(1);
            robot.armToPosition(runningOpMode);
            robot.mtrTurret.setPower(-0.3);
            duckTimer.reset();
            while(duckTimer.seconds() <= var.duckTime && robot.mtrArm.isBusy()){
                telemetry.addData("is servo running? ", robot.svoCarousel.getPower());
                telemetry.update();
                if (robot.midLimit.isPressed()){
                    robot.mtrTurret.setPower(0);
                }
            }
            robot.svoCarousel.setPower(0);
            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(runningOpMode == 3){
                drive.followTrajectory(toRedHub3);
                spitOutBlock();
                //drive.setPoseEstimate(traj.redHub3Exit);
                robot.svoIntakeTilt.setPosition(var.intakeHigh);
                robot.mtrTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.moveturret(0.3,1480);
                robot.mtrTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectoryAsync(toWarehouse1_3);
            }
            else if(runningOpMode == 2){
                drive.followTrajectory(toRedHub2);
                spitOutBlock();
                //drive.setPoseEstimate(traj.redHub2Exit);
                robot.svoIntakeTilt.setPosition(var.intakeHigh);
                robot.mtrTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.moveturret(0.3,1480);
                robot.mtrTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectoryAsync(toWarehouse1_2);
            }else if(runningOpMode == 1){
                drive.followTrajectory(toRedHub1);
                spitOutBlock();
                //drive.setPoseEstimate(traj.redHub1Exit);
                robot.svoIntakeTilt.setPosition(var.intakeHigh);
                robot.mtrTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.moveturret(0.3,1480);
                robot.mtrTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectoryAsync(toWarehouse1_1);
            }

            //TODO: add the move arm encoder positions to the runningOpMode ifs above

            //TODO: optimization lmao
            //TODO: make arm going down into a marker (either displacement or spacial)
            while (robot.mtrTurret.isBusy()) {
                telemetry.addLine("turret go brrrrr");
                telemetry.update();
                drive.update();
            }
            robot.mtrTurret.setPower(0);
            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addLine("arm go brrrrrrrrrrrrrrrrrrrrrrrrrr");
            telemetry.update();
            robot.svoIntakeTilt.setPosition(var.intakeCollect+0.2);
            while(!robot.bottomLimit.isPressed()){
                robot.mtrArm.setPower(0.7);
            }
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.movearm(0.5,150);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.mtrArm.isBusy()){

            }
            robot.mtrArm.setPower(0);


            Trajectory traj = drive.trajectoryBuilder(toWarehouse1_3.end())
                    .forward(50, FFMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(toWarehouse1_3.end())
                    .forward(50, FFMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), FFMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();


            /**
             * drive forward for consumption
             */
            robot.svoIntake.setPower(var.lessPower);
            drive.followTrajectoryAsync(traj);
            while (robot.intakeLimit.isPressed()) {
                telemetry.addLine("consuming");
                telemetry.update();
                drive.update();
                drive.updatePoseEstimate();
            }
            drive.cancelFollowing();
            intakeEnd = drive.getPoseEstimate();
            drive.setDrivePower(new Pose2d());
            drive.update();
            telemetry.addLine("consumed");
            telemetry.addData("intake end: ", intakeEnd);
            telemetry.update();
            robot.svoIntake.setPower(0);

            /**
             * has been consumed, now go to hub
             */
            //TODO: make this spline correct lmao
            Trajectory goBack = drive.trajectoryBuilder(intakeEnd, true)
                    .splineToConstantHeading(new Vector2d(-12, -47), Math.toRadians(90))
                    .build();
            drive.followTrajectory(goBack);
            //arm
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.movearm(0.7, var.thirdLvl);

            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.mtrArm.isBusy()) {

            }
            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (!robot.midLimit.isPressed()) {
                robot.mtrTurret.setPower(-0.3);
            }
            robot.mtrTurret.setPower(0);
            spitOutBlock();

            /**
             * send the turret back and arm down to collect again
             */
            drive.followTrajectory(toWarehouse1_3);
            robot.mtrTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.moveturret(0.3,1480);
            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(100);
            while (robot.mtrTurret.isBusy()) {
                telemetry.addLine("turret go brrrrr");
                telemetry.update();
            }
            robot.mtrTurret.setPower(0);
            robot.mtrTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addLine("arm go brrrrrrrrrrrrrrrrrrrrrrrrrr");
            telemetry.update();
            robot.svoIntakeTilt.setPosition(var.intakeCollect);
            while(!robot.bottomLimit.isPressed()){
                robot.mtrArm.setPower(0.7);
            }
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.movearm(0.5,150);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.mtrArm.isBusy()){

            }
            robot.mtrArm.setPower(0);

            /**
             * begin consumption AGAIN
             */
            robot.svoIntake.setPower(1);

            drive.followTrajectoryAsync(traj2);
            drive.update();
            sleep(100);
            while (robot.intakeLimit.isPressed()) {
                telemetry.addLine("consuming");
                telemetry.update();
                drive.update();
                drive.updatePoseEstimate();
            }
            drive.cancelFollowing();
            intakeEnd = drive.getPoseEstimate();
            drive.setDrivePower(new Pose2d());
            drive.update();
            telemetry.addLine("consumed");
            telemetry.addData("intake end: ", intakeEnd);
            telemetry.update();
            robot.svoIntake.setPower(0);
            Trajectory goBack2 = drive.trajectoryBuilder(intakeEnd, true)
                    .splineToConstantHeading(new Vector2d(-12, -47), Math.toRadians(90))
                    .build();
            drive.followTrajectory(goBack2);
            //arm
            robot.mtrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.movearm(0.7, var.thirdLvl);

            robot.mtrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.mtrArm.isBusy()) {

            }
            robot.mtrArm.setPower(0);
            robot.mtrArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (!robot.midLimit.isPressed()) {
                robot.mtrTurret.setPower(-0.3);
            }
            robot.mtrTurret.setPower(0);
            spitOutBlock();
            drive.followTrajectory(toWarehouse1_3);
            break;
        }
    }

    public void spitOutBlock (){
        if (runningOpMode ==3 ){
            robot.svoIntakeTilt.setPosition(var.intakeHigh);
        }
        else if (runningOpMode == 2){
            robot.svoIntakeTilt.setPosition(var.intakeMid);
        }
        else if (runningOpMode  == 1){
            robot.svoIntakeTilt.setPosition(var.intakeLow);
        }
        sleep(1000);
        robot.svoIntake.setPower(-var.lessPower);
        sleep(1500);
        robot.svoIntake.setPower(0);
    }
}
