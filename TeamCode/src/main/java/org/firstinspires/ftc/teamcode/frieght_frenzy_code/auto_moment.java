package org.firstinspires.ftc.teamcode.frieght_frenzy_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "auto moment", group = "auto")
public class auto_moment extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    //motors
    hardwareFF robot = new hardwareFF();


    Vector2d shootingPosition = new Vector2d(-3,-30);

    Vector2d noRingWobblePickUp = new Vector2d(-39,-49.5);
    Vector2d oneRingWobblePickUp = new Vector2d(-39,-49.5);
    Vector2d fourRingWobblePickUp = new Vector2d(-39,-49.75);

    Vector2d park01 = new Vector2d(9,-35);
    Vector2d park4 = new Vector2d(9,-55);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-60,-14,0);
        drive.setPoseEstimate(startPose);

        /***
         *               ODOMETRY TRAJECTORIES
         */
        /*
        Pose2d startPose = new Pose2d(-63, -14, 0);
        drive.setPoseEstimate(startPose);

        //shooting trajectory for A lol:
        Trajectory toShootA = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(
                        shootingPosition, Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        //shoot to go around ring stack lmao:
        Trajectory toShootBC = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(
                        new Vector2d(1, -4), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        shootingPosition, Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();



         */
        /***
         * NO RING
         */

        /*
        Trajectory noRingToA = drive.trajectoryBuilder(toShootA.end())
                .strafeTo(
                        new Vector2d(4, -50)
                )
                .build();


        Trajectory noRingToWobble = drive.trajectoryBuilder(noRingToA.end(), true)
                .lineToConstantHeading(
                        noRingWobblePickUp,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        Trajectory noRingToA2 = drive.trajectoryBuilder(noRingToWobble.end())
                .splineToLinearHeading(
                        new Pose2d(-3, -55, Math.toRadians(180) + 1e-6), Math.toRadians(0))
                .addTemporalMarker(1, () ->
                      robot.mtrBL.setPower(1))
                .build();


        Trajectory noRingToPark = drive.trajectoryBuilder(noRingToA2.end())
                .splineToConstantHeading(
                        new Vector2d(-18, -55), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-18, -35), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        park01, Math.toRadians(0)
                )
                .build();


         */

        /***
         * ONE RING
         */

        /*
        //spline to align with B
        Trajectory oneRingToB = drive.trajectoryBuilder(toShootBC.end())
                .lineToConstantHeading(new Vector2d(26, -27))
                .build();


        Trajectory oneRingToWobble1 = drive.trajectoryBuilder(oneRingToB.end())
                .lineToConstantHeading(
                        new Vector2d(20, -49.5),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory oneRingToWobble2 = drive.trajectoryBuilder(oneRingToWobble1.end(), true)
                .lineToConstantHeading(
                        new Vector2d(-8, -49.5),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory oneRingToWobble3 = drive.trajectoryBuilder(oneRingToWobble2.end(), true)
                .lineToConstantHeading(
                        oneRingWobblePickUp,
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        Trajectory oneRingToShoot = drive.trajectoryBuilder(oneRingToWobble3.end())
                //go pick up da ring!!!
                .addDisplacementMarker(() -> robot.mtrIntake.setPower(1))
                .splineToConstantHeading(
                        new Vector2d(-39, -33), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        new Vector2d(-18, -33), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(
                        shootingPosition, Math.toRadians(0)
                )
                .addDisplacementMarker(() -> {
                    drive.turn(Math.toRadians(shootCorrection));
                    waitFor(1);
                    robot.svoMagLift.setPosition(var.magUp);
                    shootOne();
                    robot.svoMagLift.setPosition(var.magDown);
                })
                .addTemporalMarker(1, () ->
                        robot.mtrWobble.setPower(counterWobblePower))
                .build();


        Trajectory oneRingToB2 = drive.trajectoryBuilder(oneRingToShoot.end())
                //then turn around to slap da wobble down umu
                .splineToLinearHeading(
                        new Pose2d(18, -30, Math.toRadians(180) + 1e-6), Math.toRadians(0))
                .build();


        Trajectory oneRingToPark = drive.trajectoryBuilder(oneRingToB2.end())
                .lineToConstantHeading(
                        park01
                )

                .build();


         */
        /***
         * FOUR RINGS
         */
/*
        //spline to align with C
        Trajectory fourRingsToC = drive.trajectoryBuilder(toShootBC.end().plus(new Pose2d(0, 0, Math.toRadians(shootAngleCorrection))))
                .splineToConstantHeading(
                        new Vector2d(50, -47), Math.toRadians(0)
                )
                .build();

        Trajectory fourRingsToWobble1 = drive.trajectoryBuilder(fourRingsToC.end())
                .lineToConstantHeading(
                        new Vector2d(-20, -49.75)
                )
                .build();
        Trajectory fourRingsToWobble2 = drive.trajectoryBuilder(fourRingsToWobble1.end())
                .addDisplacementMarker(() ->
                        robot.mtrIntake.setPower(1))
                .lineToConstantHeading(
                        fourRingWobblePickUp,
                        SampleMecanumDrive.getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory fourRingsToShoot10 = drive.trajectoryBuilder(fourRingsToWobble2.end().plus(new Pose2d (0,0, Math.toRadians(60))))

                .lineToConstantHeading(
                        new Vector2d(-27, -44)
                )
                .build();
        Trajectory fourRingsToShoot11 = drive.trajectoryBuilder(fourRingsToShoot10.end())
                .lineToConstantHeading(
                        new Vector2d(-25, -42)
                )
                .build();
        Trajectory fourRingsToShoot12 = drive.trajectoryBuilder(fourRingsToShoot11.end().plus(new Pose2d (0,0, Math.toRadians(-60))))
                .lineToConstantHeading(
                        new Vector2d(-3, -41)
                )
                .build();




 */
        /***
         * here we go code for ring stack and park
         */
/*
        Trajectory fourRingsToC2 = drive.trajectoryBuilder(fourRingsToShoot12.end().plus(new Pose2d (0,0, Math.toRadians(-179.9))),true)
                //then turn around to slap da wobble down umu
                .lineToConstantHeading(
                        new Vector2d(44, -54)
                )
                .build();


        Trajectory fourRingsToPark = drive.trajectoryBuilder(fourRingsToC2.end())
                .lineToConstantHeading(
                        park4
                )

                .build();
*/

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {




            telemetry.update();


        }
    }

}
