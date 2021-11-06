package org.firstinspires.ftc.teamcode.frieght_frenzy_code;
 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 
@TeleOp
public class AllianceColor extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;
    static double power = 1;
    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");
        if (color.red() > 200){
            return;
        }
        else if (color.blue() > 200){
            power = -power;
        }
        // Wait for the Play button to be pressed
        waitForStart();
 
        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }
    }
}