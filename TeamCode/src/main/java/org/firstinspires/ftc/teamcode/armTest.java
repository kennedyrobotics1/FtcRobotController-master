package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name= "armTest", group= "Linear OpMode")

public class DriveTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;

    @Override
    public void runOpMode() {
        motor0  = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

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

            motor0Power =  (y + x + r) / denominator;
            motor1Power = (y - x - r) / denominator;
            motor2Power = (y - x + r) / denominator;
            motor3Power = (y + x - r) / denominator;

            motor0.setPower(motor0Power);
            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);
            motor3.setPower(motor3Power);
        }

    }
}