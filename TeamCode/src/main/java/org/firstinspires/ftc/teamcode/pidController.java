package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class pidController extends LinearOpMode{
    DcMotorEx leftMotorFront;
    DcMotorEx leftMotorBack;
    DcMotorEx rightMotorFront;
    DcMotorEx rightMotorBack;
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    @Override
    public void runOpMode() throws  InterruptedException {
        leftMotorFront = hardwareMap.get(DcMotorEx.class, "lmf");
        leftMotorBack = hardwareMap.get(DcMotorEx.class, "lmb");
        rightMotorFront = hardwareMap.get(DcMotorEx.class, "rmf");
        rightMotorBack = hardwareMap.get(DcMotorEx.class, "rmb");
        leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            double lmfPower = PIDControl(100, leftMotorFront.getCurrentPosition());
            leftMotorFront.setPower(lmfPower);
            double rmfPower = PIDControl(100, rightMotorFront.getCurrentPosition());
            rightMotorFront.setPower(rmfPower);
            leftMotorBack.setPower(lmfPower);
            rightMotorBack.setPower(rmfPower);
        }
    }

    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}
