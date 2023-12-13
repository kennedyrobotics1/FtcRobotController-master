package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

@Autonomous(name="RyanAutonomousLinear", group="Linear OpMode")
public class RyanAutonomousLinear extends LinearOpMode {
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private ElapsedTime runtime = new ElapsedTime();

    double setError0;
    double setError1;
    double setError2;
    double setError3;
    double kp0 = 1.0 / 1500;
    double kp1 = 1.0 / 1500;
    double kp2 = 1.0 / 1500;
    double kp3 = 1.0 / 1500;
    double start0 = 0;
    double start1 = 0;
    double start2 = 0;
    double start3 = 0;
    double setPoint0 = 0;
    double setPoint1 = 0;
    double setPoint2 = 0;
    double setPoint3 = 0;
    double position0;
    double position1;
    double position2;
    double position3;
    double error0;
    double error1;
    double error2;
    double error3;
    double motorPower0;
    double motorPower1;
    double motorPower2;
    double motorPower3;

    @Override
    public void runOpMode() {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        position0 = motor0.getCurrentPosition() - start0;
        position1 = motor1.getCurrentPosition() - start1;
        position2 = motor2.getCurrentPosition() - start2;
        position3 = motor3.getCurrentPosition() - start3;

        telemetry.addData("position0 ", position0);
        telemetry.update();

        waitForStart();
        telemetry.addData("opModeIsActice ", opModeIsActive());
        telemetry.addData("position1 ", position1);
        telemetry.update();
        start0 = motor0.getCurrentPosition();
        start1 = motor1.getCurrentPosition();
        start2 = motor2.getCurrentPosition();
        start3 = motor3.getCurrentPosition();
        boolean move1 = true;
        boolean move2 = false;

        position();
        telemetry.addData("opModeIsActice ", opModeIsActive());
        telemetry.addData("position1 ", position1);
        telemetry.update();
        while (opModeIsActive()) {

            telemetry.addData("setPoint0 ", setPoint0);
            telemetry.update();



            if (move1) {
                setPoint0 = 2000;
                setPoint1 = 2000;
                setPoint2 = 2000;
                setPoint3 = 2000;
                if(position0 < error0){
                    motorPower0 = position0 * kp0;
                    motorPower1 = position1 * kp1;
                    motorPower2 = position2 * kp2;
                    motorPower3 = position3 * kp3;
                } else {
                    motorPower0 = error0 * kp0;
                    motorPower1 = error1 * kp1;
                    motorPower2 = error2 * kp2;
                    motorPower3 = error3 * kp3;
                }

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower2);
                motor3.setPower(motorPower3);
                position();
                if ((error0 <= 100 && error0 >= -100) && (error1 <= 100 && error1 >= -100)) {
                    move1 = false;
                    move2 = true;
                }
                telemetry.addData("kp0 ", kp0);
                telemetry.addData("error0 ", error0);
                telemetry.addData("error1 ", error1);
                telemetry.update();
            }


            else if (move2) {
                setPoint0 = -100;
                setPoint1 = 100;
                setPoint2 = -100;
                setPoint3 = 100;
                /*if(position0 < error0){
                    motorPower0 = position0 * kp0;
                    motorPower1 = position1 * kp1;
                    motorPower2 = position2 * kp2;
                    motorPower3 = position3 * kp3;
                } else {
                    motorPower0 = error0 * kp0;
                    motorPower1 = error1 * kp1;
                    motorPower2 = error2 * kp2;
                    motorPower3 = error3 * kp3;
                }*/

                motor0.setPower(error0 * kp0);
                motor1.setPower(error1 * kp1);
                motor2.setPower(error2 * kp2);
                motor3.setPower(error3 * kp3);
                position();
                if ((error0 <= 10 && error0 >= -10) && (error1 <= 10 && error1 >= -10)) {
                    move2 = false;
                }
            }
        }
    }
    public void position() {
        boolean first = true;
        position0 = motor0.getCurrentPosition() - start0;
        position1 = motor1.getCurrentPosition() - start1;
        position2 = motor2.getCurrentPosition() - start2;
        position3 = motor3.getCurrentPosition() - start3;
        error0 = setPoint0 - position0;
        error1 = setPoint1 - position1;
        error2 = setPoint2 - position2;
        error3 = setPoint3 - position3;

        if (first) {
            setError0 = error0;
            setError1 = error1;
            setError2 = error2;
            setError3 = error3;
            first = false;
        }

    }
}


