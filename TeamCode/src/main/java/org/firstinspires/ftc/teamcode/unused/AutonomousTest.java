///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
///*
// * This OpMode illustrates the concept of driving a path based on encoder counts.
// * The code is structured as a LinearOpMode
// *
// * The code REQUIRES that you DO have encoders on the wheels,
// *   otherwise you would use: RobotAutoDriveByTime;
// *
// *  This code ALSO requires that the drive Motors have been configured such that a positive
// *  power command moves them forward, and causes the encoders to count UP.
// *
// *   The desired path in this example is:
// *   - Drive forward for 48 inches
// *   - Spin right for 12 Inches
// *   - Drive Backward for 24 inches
// *   - Stop and close the claw.
// *
// *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
// *  that performs the actual movement.
// *  This method assumes that each movement is relative to the last stopping place.
// *  There are other ways to perform encoder based moves, but this method is probably the simplest.
// *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
// */
//
//@Autonomous(name="AutonomousTest", group="Robot")
//public class AutonomousTest extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    private DcMotor motor0 = null;
//    private DcMotor motor1 = null;
//    private DcMotor motor2 = null;
//    private DcMotor motor3 = null;
//    private ElapsedTime runtime = new ElapsedTime();
//    private DistanceSensor sensorDistance;
//
//    // Calculate the COUNTS_PER_INCH for your specific drive train.
//    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
//    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
//    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
//    // This is gearing DOWN for less speed and more torque.
//    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//    static final double     COUNTS_PER_MOTOR_REV    = 5700.4 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
//    static final double     DRIVE_SPEED             = 0.6;
//    static final double     TURN_SPEED              = 0.5;
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize the drive system variables.
//        motor0  = hardwareMap.get(DcMotor.class, "motor0");
//        motor1 = hardwareMap.get(DcMotor.class, "motor1");
//        motor2  = hardwareMap.get(DcMotor.class, "motor2");
//        motor3 = hardwareMap.get(DcMotor.class, "motor3");
//        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
//
//
//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
//        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        motor0.setDirection(DcMotor.Direction.REVERSE);
//        motor2.setDirection(DcMotor.Direction.REVERSE);
//
//        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Starting at",  "%7d :%7d",
//                          motor0.getCurrentPosition(),
//                          motor1.getCurrentPosition(),
//                          motor2.getCurrentPosition(),
//                          motor3.getCurrentPosition());
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // Step through each leg of the path,
//        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  40,  40, 0.77);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 0.36);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, 24, 24, 0.8);  // S3: Reverse 24 Inches with 4 Sec timeout
//        encoderDrive(TURN_SPEED,   -12, 12, 0.35);
//        encoderDrive(DRIVE_SPEED, 24, 24, 0.425);
//        encoderDrive(TURN_SPEED,   12, -12, 0.4);
//        encoderDrive(DRIVE_SPEED, 24, 24, 0.225);
//        //encoderDrive(DRIVE_SPEED,  48,  48, 0.75);
//
//        if(sensorDistance.getDistance(DistanceUnit.INCH) <= 12){
//            encoderDrive(DRIVE_SPEED,  0,  0, 0);
//        }
//
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);  // pause to display final telemetry message.
//    }
//
//    /*
//     *  Method to perform a relative move, based on encoder counts.
//     *  Encoders are not reset as the move is based on the current position.
//     *  Move will stop if any of three conditions occur:
//     *  1) Move gets to the desired position
//     *  2) Move runs out of time
//     *  3) Driver stops the OpMode running.
//     */
//    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
//        int newMotor0Target;
//        int newMotor1Target;
//        int newMotor2Target;
//        int newMotor3Target;
//
//        // Ensure that the OpMode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newMotor1Target = motor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newMotor0Target = motor0.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            newMotor3Target = motor3.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newMotor2Target = motor2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            motor1.setTargetPosition(newMotor1Target);
//            motor0.setTargetPosition(newMotor0Target);
//            motor3.setTargetPosition(newMotor3Target);
//            motor2.setTargetPosition(newMotor2Target);
//
//            // Turn On RUN_TO_POSITION
//            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            motor0.setPower(Math.abs(speed));
//            motor1.setPower(Math.abs(speed));
//            motor2.setPower(Math.abs(speed));
//            motor3.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motor0.isBusy() && motor1.isBusy() && motor2.isBusy() && motor3.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Running to ", " %7d :%7d", newMotor1Target, newMotor0Target);
//                telemetry.addData("Currently at",  " at %7d :%7d",
//                                            motor1.getCurrentPosition(), motor0.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            motor0.setPower(0);
//            motor1.setPower(0);
//            motor2.setPower(0);
//            motor3.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            sleep(250);   // optional pause after each move.
//        }
//    }
//}
