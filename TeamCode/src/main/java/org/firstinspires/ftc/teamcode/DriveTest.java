package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.Timer;

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
    //private DcMotor arm0 = null;
    //private DcMotor arm1 = null;
    //private DistanceSensor sensorDistance;
    //private ColorSensor colorSensor;

    public final static double ARM_HOME = 0.0;
    double ARM_SPEED = 0.75;

    double armPosition = 0;

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;

    double start0;
    double start1;
    double start2;
    double start3;
    double headingError;
    double setYaw;
    double kpA = 1.0/100;



    @Override
    public void runOpMode() {
        motor0  = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        //arm0 = hardwareMap.get(DcMotor.class, "arm0");
        //arm1 = hardwareMap.get(DcMotor.class, "arm1");

        //arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward

        /*boolean bLedOn = false;
        colorSensor.enableLed(bLedOn);*/

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        //arm1.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        timer driveTimer = new timer();


        start0 = motor0.getCurrentPosition();
        start1 = motor1.getCurrentPosition();
        start2 = motor2.getCurrentPosition();
        start3 = motor3.getCurrentPosition();
        imu.resetYaw();
        runtime.reset();

        while (opModeIsActive()) {
            driveTimer.updateTime();
            // Setup a variable for each drive wheel to save power level for telemetry
            double motor0Power, motor1Power, motor2Power, motor3Power;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            speedRamp XRamp = new speedRamp(0.000000005);
            speedRamp YRamp = new speedRamp(0.000000005);
            double x = detectDirection(gamepad1.left_stick_x) *  XRamp.operator(gamepad1.left_stick_x, driveTimer);
            double y = -1 * detectDirection(gamepad1.left_stick_y) * YRamp.operator(gamepad1.left_stick_y, driveTimer);
            double r = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);

           // x = joyStickFloor(x, .2, .4, .3);
            //y = joyStickFloor(y, .2, .4, .3);

            motor0Power = (y + x + r) / denominator;
            motor1Power = (y - x - r) / denominator;
            motor2Power = (y - x + r) / denominator;
            motor3Power = (y + x - r) / denominator;
            /*if(gamepad2.a){
                armPosition = pidController.PIDControl(-50, arm0.getCurrentPosition());
                arm0.setTargetPosition(arm0.getCurrentPosition() - 20);
                arm1.setTargetPosition(arm0.getCurrentPosition() - 20);
                arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if(gamepad2.y){
                armPosition = pidController.PIDControl(50, arm0.getCurrentPosition());
                arm0.setTargetPosition(arm0.getCurrentPosition() + 20);
                arm1.setTargetPosition(arm0.getCurrentPosition() + 20);
                arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            arm0.setPower(0.5);
            arm1.setPower(0.5);
        */
            /*double k = -gamepad2.left_stick_y;
            arm0.setPower(0.70 * k);
            arm1.setPower(0.70 * k);*/
            if(gamepad1.dpad_up){
                motor0.setPower(0.8);
                motor1.setPower(0.8);
                motor2.setPower(0.8);
                motor3.setPower(0.8);
            }
            if(gamepad1.dpad_down){
                motor0.setPower(-0.8);
                motor1.setPower(-0.8);
                motor2.setPower(-0.8);
                motor3.setPower(-0.8);
            }

            if(gamepad1.right_bumper){
                position();
                setYaw = 90;

                motor0.setPower(-(headingError * kpA));
                motor1.setPower(headingError * kpA);
                motor2.setPower(-(headingError * kpA));
                motor3.setPower(headingError * kpA);
            }


            /*while(sensorDistance.getDistance(DistanceUnit.INCH) <= 25){
                motor0.setPower(-0.5);
                motor1.setPower(-0.5);
                motor2.setPower(-0.5);
                motor3.setPower(-0.5);*/
            int A = (65280 >> 24) & 0xff; // or color >>> 24
            int R = (65280 >> 16) & 0xff;
            int G = (65280 >>  8) & 0xff;
            int B = (65280      ) & 0xff;

            /*telemetry.addData("ArmPosition: ", (int) armPosition);
            telemetry.addData("Arm0 current position:", arm0.getCurrentPosition());

            telemetry.addData("Distance: ", sensorDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("ARGB: ", colorSensor.argb());
            telemetry.addData("Red: ", R);
            telemetry.addData("Green: ", G);
            telemetry.addData("Blue: ", B);
            telemetry.addData("Alpha: ", A);
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addData("Green: ", colorSensor.green());
            telemetry.addData("Blue: ", colorSensor.blue());
            telemetry.addData("position0 ", motor0.getCurrentPosition() - start0);
            telemetry.addData("position1 ", motor1.getCurrentPosition() - start1);
            telemetry.addData("position2 ", motor2.getCurrentPosition() - start2);
            telemetry.addData("position3 ", motor3.getCurrentPosition() - start3);*/
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();


           /* telemetry.addData("Yaw: ", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("InputX ", gamepad1.left_stick_x);
            telemetry.addData("X ", x);

            telemetry.update();*/
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
    public void position() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        headingError = setYaw - orientation.getYaw(AngleUnit.DEGREES);
    }
    public class speedRamp{
        private double m_pctPerSecond;
        private double m_lastOutput;
        private double m_lastUpdateTime;

        public speedRamp(double pctPerSecond){
            m_pctPerSecond = pctPerSecond;
            m_lastOutput = 0;
            m_lastUpdateTime = System.currentTimeMillis();
        }
        public double operator(double newSample, timer driveTimer){
            double now = driveTimer.getTime();
            double currentDelta = now - m_lastUpdateTime;
            double seconds = currentDelta / 1000.0;


            double changeDirection = Math.copySign(newSample - m_lastOutput, 1.0);
            double desiredChangeMagnitude = Math.abs(newSample - m_lastOutput);
            double maxTimeRampMagnitude = m_pctPerSecond * seconds;
            double delta = changeDirection * Math.min(desiredChangeMagnitude, maxTimeRampMagnitude);


            if(false){
                telemetry.addData("Now ", now);
                telemetry.addData("currentDelta", currentDelta);
                telemetry.addData("changeDirection ", changeDirection);
                telemetry.addData("seconds ", seconds);
                telemetry.addData("desired ", desiredChangeMagnitude);
                telemetry.addData("maxTime ", maxTimeRampMagnitude);
                telemetry.addData("delta ", delta);
                telemetry.update();
            }
            m_lastOutput += delta;
            return m_lastOutput;
        }
    }
    public class timer{

        public double previousTime;
        public double deltaTime;
        public double startTime;
        public timer(){
            startTime = System.nanoTime();

        }
        public double getTime(){
            return System.nanoTime();
        }
        public void updateTime() {
            previousTime = time;
            time = ((double) (System.nanoTime() - startTime) ) / Math.pow(10,9);
            deltaTime = time - previousTime;
        }
    }
    public int detectDirection(double input){
        if(input<0){
            return -1;
        }
        return 1;
    }

  /*  public double joyStickFloor(double input, double threshold, double deadZone, double motorValue){
        if(deadZone < input && input < threshold) {
            return motorValue;
        }
        return input;
    }*/
}