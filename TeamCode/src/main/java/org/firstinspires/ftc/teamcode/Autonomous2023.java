package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Autonomous baby", group="Linear Opmode")
public class Autonomous2023 extends LinearOpMode {


    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // Go to your motor vendor website to determine your motor's WHEEL_DIAMETER
    //  Circumfrence of the wheel
    // Calculate the COUNTS_PER_INCH for your specific drive train.

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_CM = 9.8;     // For figuring circumference
    static final double distance_per_rev = (WHEEL_DIAMETER_CM * Math.PI);

    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV);


    private MainConfig2023 config = new MainConfig2023();

    @Override
    public void runOpMode() {

        waitForStart();
        config.runtime.reset();


        while (opModeIsActive()) {


            telemetry.addData("Status", "Run Time: " + config.runtime.toString());
        }


    }

    public void Drive_Train(double speed, double distance, double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;


        if (opModeIsActive()) {

            int distance_travel = (int) (distance / distance_per_rev * COUNTS_PER_MOTOR_REV);
            newLeftTarget = config.leftDrive.getCurrentPosition();
            newRightTarget = config.rightDrive.getCurrentPosition();
            newLeftTarget2 = config.leftDrive.getCurrentPosition();
            newRightTarget2 = config.rightDrive2.getCurrentPosition();

            config.leftDrive.setTargetPosition(newLeftTarget);
            config.rightDrive.setTargetPosition(newRightTarget);
            config.leftDrive2.setTargetPosition(newLeftTarget2);
            config.rightDrive2.setTargetPosition(newRightTarget2);

            config.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            config.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            config.runtime.reset();
            config.leftDrive.setPower(Math.abs(speed));
            config.rightDrive.setPower(Math.abs(speed));
            config.leftDrive2.setPower(Math.abs(speed));
            config.rightDrive2.setPower(Math.abs(speed));

            /**
            while (opModeIsActive() &&
                    (config.runtime.seconds() < timeoutS)
            {

            }
             */

        }
    }
}





