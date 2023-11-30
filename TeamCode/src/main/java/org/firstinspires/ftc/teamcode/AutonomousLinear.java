package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

@Autonomous(name="Autonomous", group="Linear OpMode")
public class AutonomousLinear extends LinearOpMode {
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DistanceSensor sensorDistance;

    double setError0;
    double setError1;
    double setError2;
    double setError3;
    double kp0 = 1.0 / setError0;
    double kp1 = 1.0 / setError1;
    double kp2 = 1.0 / setError2;
    double kp3 = 1.0 / setError3;
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

    @Override
    public void runOpMode() {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;



        position0 = motor0.getCurrentPosition() - start0;
        position1 = motor1.getCurrentPosition() - start1;
        position2 = motor2.getCurrentPosition() - start2;
        position3 = motor3.getCurrentPosition() - start3;


        waitForStart();

        start0 = motor0.getCurrentPosition();
        start1 = motor1.getCurrentPosition();
        start2 = motor2.getCurrentPosition();
        start3 = motor3.getCurrentPosition();

        position();
        setPoint0 = 100;
        setPoint1 = 100;
        setPoint2 = 100;
        setPoint3 = 100;
        boolean move1 = true;
        while(move1){
            motor0.setPower(error0 * kp0);
            motor1.setPower(error1 * kp1);
            motor2.setPower(error2 * kp2);
            motor3.setPower(error3 * kp3);
            position();
            if(error0 == 0 && error1 == 0){
                move1 = false;
            }
        }

        setPoint0 = 25;
        setPoint1 = 25;
        setPoint2 = 25;
        setPoint3 = 25;
        position();
        boolean move2 = true;
        while(move2){
            motor0.setPower(error0 * kp0);
            motor1.setPower(error1 * kp1);
            motor2.setPower(error2 * kp2);
            motor3.setPower(error3 * kp3);
            position();
            if(error0 == 0 && error1 == 0){
                move2 = false;
            }
        }

        setPoint0 = -25;
        setPoint1 = -25;
        setPoint2 = 25;
        setPoint3 = 25;
        position();
        boolean move3 = true;
        while(move3){
            motor0.setPower(error0 * kp0);
            motor1.setPower(error1 * kp1);
            motor2.setPower(error2 * kp2);
            motor3.setPower(error3 * kp3);
            position();
            if(error0 == 0 && error1 == 0){
                move3 = false;
            }
        }
    }
    public void position(){
        boolean first = true;
        while(opModeIsActive()){
            kp0 = 1.0 / setError0;
            kp1 = 1.0 / setError1;
            kp2 = 1.0 / setError2;
            kp3 = 1.0 / setError3;

            position0 = motor0.getCurrentPosition() - start0;
            position1 = motor1.getCurrentPosition() - start1;
            position2 = motor2.getCurrentPosition() - start2;
            position3 = motor3.getCurrentPosition() - start3;
            error0 = setPoint0 - position0;
            error1 = setPoint1 - position1;
            error2 = setPoint2 - position2;
            error3 = setPoint3 - position3;
            if(first){
                setError0 = error0;
                setError1 = error1;
                setError2 = error2;
                setError3 = error3;
                first = false;
            }
        }
    }
}
