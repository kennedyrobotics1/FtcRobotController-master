package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name= "DriveTest", group= "Linear OpMode")

public class DriveTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    private DistanceSensor sensorDistance;
    private ColorSensor colorSensor;

    double ARM_SPEED = 0.75;
    @Override
    public void runOpMode() {
        motor0  = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        arm0 = hardwareMap.get(DcMotor.class, "arm0");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        boolean bLedOn = true;
        colorSensor.enableLed(bLedOn);

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        arm1.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double motor0Power, motor1Power, motor2Power, motor3Power;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double x = gamepad1.left_stick_x;
            double y  =  -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
            /*if(x <= 1 && x >= 0.9){
                x = 1;
                y = 0;
            }
            if(x > -0.8 && x <= -0.5 && y > 0.2 && y < 0.5){
                x = -0.7;
                y = 0.3;
            }
            if(x < 0.8 && x >= 0.5 && y < -0.2 && y > -0.5){
                x = 0.7;
                y = -0.3;
            }
            if(x > -0.8 && x <= -0.5 && y < -0.2 && y > -0.5){
                x = -0.7;
                y = -0.3;
            }
            if(x < 0.8 && x >= 0.5 && y > 0.2 && y < 0.5){
                x = 0.7;
                y = 0.3;
            }
            if(x <= 1 && x >= 0.9){
                x = 1;
                y = 0;
            }
            if(y > -0.8 && y <= -0.5 && x > 0.2 && x < 0.5){
                y = -0.7;
                x = 0.3;
            }
            if(y < 0.8 && y >= 0.5 && x < -0.2 && x > -0.5){
                y = 0.7;
                x = -0.3;
            }
            if(y > -0.8 && y <= -0.5 && x < -0.2 && x > -0.5){
                y = -0.7;
                x = -0.3;
            }
            if(y < 0.8 && y >= 0.5 && x > 0.2 && x < 0.5){
                y = 0.7;
                x = 0.3;
            }

            if(x >= -1 && x <= -0.8){
                x = -1;
                y = 0;
            }
            if(y <= 1 && y >= 0.8){
                y = 1;
                x = 0;
            }
            if(y >= -1 && y <= -0.9){
                y = -1;
                x = 0;
            }*/
            motor0Power = (y + x + r) / denominator;
            motor1Power = (y - x - r) / denominator;
            motor2Power = (y - x + r) / denominator;
            motor3Power = (y + x - r) / denominator;

            if(gamepad1.a){
                arm0.setPower(ARM_SPEED);
                arm1.setPower(ARM_SPEED);
            } else if(gamepad1.y){
                arm0.setPower(-0.02 * ARM_SPEED);
                arm1.setPower(-0.02 * ARM_SPEED);
            }



            while(sensorDistance.getDistance(DistanceUnit.INCH) <= 25){
                motor0.setPower(-1);
                motor1.setPower(-1);
                motor2.setPower(-1);
                motor3.setPower(-1);
            }


            int A = (65280 >> 24) & 0xff; // or color >>> 24
            int R = (65280 >> 16) & 0xff;
            int G = (65280 >>  8) & 0xff;
            int B = (65280      ) & 0xff;



            telemetry.addData("", colorSensor.argb());
            telemetry.addData("Red: ", R);
            telemetry.addData("Green: ", G);
            telemetry.addData("Blue: ", B);
            telemetry.addData("Alpha: ", A);
            telemetry.addData("Distance: ", sensorDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addData("Green: ", colorSensor.green());3
            telemetry.addData("Blue: ", colorSensor.blue());
            telemetry.addData("Motor0 Power: ", motor0Power);
            telemetry.addData("Motor1 Power: ", motor1Power);
            telemetry.addData("Motor2 Power: ", motor2Power);
            telemetry.addData("Motor3 Power: ", motor3Power);
            telemetry.update();
            //slow mode
            if(gamepad1.left_bumper){
                motor0.setPower(0.6 * motor0Power);
                motor1.setPower(0.6 * motor1Power);
                motor2.setPower(0.6 * motor2Power);
                motor3.setPower(0.6 * motor3Power);
            } else {
                motor0.setPower(motor0Power);
                motor1.setPower(motor1Power);
                motor2.setPower(motor2Power);
                motor3.setPower(motor3Power);
            }
        }

    }
}
