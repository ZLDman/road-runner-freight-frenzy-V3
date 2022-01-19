package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Robot {

    public DcMotorEx rotate;
    public DcMotor lift;
    public DcMotor intake;
    public DcMotor car;
    public Servo s;

    public static double sOpen = 0.4;
    public static double sClosed = 0.15 ;

    public double rotateTargetSpeed = 0;

    public static double carMaxSpeed = 0.75;

    public enum target {
        place, intake, manuel
    }

    double rotateTargetPower = 0;

    target t = target.manuel;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    public Robot(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotor.class, "lift");
        rotate = hardwareMap.get(DcMotorEx.class, "rotate");
        intake = hardwareMap.get(DcMotor.class, "intake");
        car = hardwareMap.get(DcMotor.class, "car");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        s = hardwareMap.get(Servo.class,"s");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor .class, "color");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor .class, "color");

        //rotate.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(2.5,0.1,0.2,0.5));
    }

    void openClaw(){
        s.setPosition(sOpen);
    }
    void closeClaw(){
        s.setPosition(sClosed);
    }

    void setLiftSpeed(double s){
        if(t == target.manuel) {
            lift.setPower(s);
        }
    }

    void setLiftPos(int p){
        lift.setTargetPosition(p);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        while(lift.isBusy()){

        }
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setRotateSpeed(double s){
        if(t == target.manuel) {
            if (s > rotate.getPower())
                rotate.setPower(rotate.getPower() + 0.05);
            if (s < rotate.getPower())
                rotate.setPower(rotate.getPower() - 0.05);
        }
    }

    void setRotatePos(int p){
        rotate.setTargetPosition(p);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate.setPower(0.5);
        while(rotate.isBusy()){

        }
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void setIntakeSpeed(double s){
        intake.setPower(s);
    }

    int getIntakePos(){
        return intake.getCurrentPosition();
    }

    /**
     * @param s speed min: 0 Max: 1
     */
    void setCarSpeed(double s){
        car.setPower(s * carMaxSpeed);
    }


    double getColor(int channel) {

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.

        if (channel == -1) return sensorDistance.getDistance(DistanceUnit.INCH);
        if (channel == 1) return sensorColor.red();
        if (channel == 2) return sensorColor.green();
        if (channel == 3) return sensorColor.blue();
        return sensorColor.alpha();


    }

    public void setTarget(target tar) {
        t = tar;
    }

    public void updateTarget(){

        int r;//initialise variables
        int l;

        if(t == target.manuel){ //sets controls to manual
            return;
        }

        // sets target position for placing freight on top level of shipping hub
        else if (t == target.place){
            l = 800;
            r = 1940;
        }

        //set target position for intaking freight
        else{
            l = 0;
            r = 0;
        }

        //only rotate when lift is up
        if(lift.getCurrentPosition() > 500) {

            // stop when we have reached target
            if (Math.abs(r - rotate.getCurrentPosition()) < 100) rotate.setPower(0);

            else if (r > rotate.getCurrentPosition())
                rotateTargetPower = 0.75;
            else if (r < rotate.getCurrentPosition())
                rotateTargetPower = -0.75;

            if (rotateTargetPower > rotate.getPower())
                rotate.setPower(rotate.getPower() + 0.1);
            if (rotateTargetPower < rotate.getPower())
                rotate.setPower(rotate.getPower() - 0.1);
        }

        // stop when we have reached target
        if(Math.abs(l - lift.getCurrentPosition()) < 50){
            lift.setPower(0);
        }

            // i don't think we need any acceleration for the lift motor
        else if(l > lift.getCurrentPosition())
            lift.setPower(1);
        else if(l < lift.getCurrentPosition())
            //only go down when arm is in
            if(rotate.getCurrentPosition() < 200) {
                lift.setPower(-1);
            }
    }
}
