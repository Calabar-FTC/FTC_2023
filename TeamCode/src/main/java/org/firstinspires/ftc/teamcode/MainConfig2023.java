package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MainConfig2023
{

    // Declare OpMode members.
     ElapsedTime runtime = new ElapsedTime();
     DcMotor leftDrive = null;
     DcMotor rightDrive = null;
     DcMotor leftDrive2 = null;
     DcMotor rightDrive2 = null;
    DcMotor linslide_left = null;
    DcMotor  linslide_right = null;
     Servo servo1;
    Servo servo2;

     HardwareMap hardMap;
     NormalizedColorSensor colorSensor;

    static double wheel_diameter = 9.8;


    public void TotalHardeware (HardwareMap hardMap)
    {
        this.hardMap = hardMap;
        colorSensor = this.hardMap.get(NormalizedColorSensor.class, "sensor_color");
        //colorSensor = this.hardMap.get(ColorSensor.class, "color_distance");


        leftDrive = hardMap.get(DcMotor.class, "left_drive");
        rightDrive = hardMap.get(DcMotor.class, "right_drive");
        leftDrive2 = hardMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hardMap.get(DcMotor.class, "right_drive2");

        linslide_left = hardMap.get(DcMotor.class, "arm_right");
        linslide_right = hardMap.get(DcMotor.class, "arm_left");

        servo1 = hardMap.get(Servo.class, "claw_right");
        servo2 = hardMap.get(Servo.class, "claw_left");

        colorSensor = hardMap.get(NormalizedColorSensor.class, "sensor_color");
        //sensorRange = hardMap.get(DistanceSensor.class, "sensor_range");



        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        linslide_left.setDirection(DcMotor.Direction.REVERSE);
        linslide_right.setDirection(DcMotor.Direction.FORWARD);

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
