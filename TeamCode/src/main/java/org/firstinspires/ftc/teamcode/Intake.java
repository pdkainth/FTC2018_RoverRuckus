package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private static final double FULL_POWER = 1.0;
    private static final double NO_POWER = 0.0;
    private CRServo intakeServo;
    private double power;

    private Servo transferServo;
    private double currentPosition;
    private static final double DOWN_POSITION = 0.325;
    private static final double UP_POSITION = 0.675;
    private ServoControllerEx controller;
    private int               port;

    private ElapsedTime elapsedTime;
    private ElapsedTime waitTime;
    private int pulseCnt = 0;

    public void init(HardwareMap hardwareMap){
      intakeServo = hardwareMap.get(CRServo.class, "CRservo0");
      intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

      transferServo = hardwareMap.get(Servo.class, "Servo1");
      transferServo.setDirection(Servo.Direction.FORWARD);
      controller = (ServoControllerEx) transferServo.getController();
      port = transferServo.getPortNumber();

      elapsedTime = new ElapsedTime();
      waitTime    = new ElapsedTime();
      elapsedTime.reset();

      stop();
      goUp();
    }

    public void turnOn(){
      power = FULL_POWER;
      intakeServo.setPower(power);
    }

    public void turnOff(){
      power = NO_POWER;
      intakeServo.setPower(power);
    }

    public boolean isOff() {
      return (power == NO_POWER);
    }

    public void pulse(float pulsePower){
      intakeServo.setPower(pulsePower);
      waitTime.reset();
      while (waitTime.milliseconds() < 100.0);
      turnOff();
      pulseCnt++;
    }

    public void goUp(){
      currentPosition = UP_POSITION;
      applyTransferPos();
    }

    public void goDown(){
      currentPosition = DOWN_POSITION;
      applyTransferPos();
    }

    public void applyTransferPos() {
      controller.setServoPwmEnable(port);
      transferServo.setPosition(currentPosition);
      elapsedTime.reset();
    }

    public void update(Telemetry telemetry){

      boolean on = controller.isServoPwmEnabled(port);

      if (on) {
        if (elapsedTime.milliseconds() > 2000.0) {
          controller.setServoPwmDisable(port);
          on = false;
        }
      }

      telemetry.addData("IntakeSpinner","power %.2f pulse %d", power, pulseCnt);
      telemetry.addData("TransferServo", "position" + currentPosition + " power " + on);
    }

    public void stop(){
      power = NO_POWER;
      intakeServo.setPower(NO_POWER);
      controller.setServoPwmDisable(port);
    }
}
