package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PhoneTilt {

  public static final double PHONE_UP_POS = 0.5;

  // With up mount
  //public static final double PHONE_TILT_DEG_VUFORIA = 10.5;
  //public static final double PHONE_TILT_DEG_TENSOR_FLOW = 25.0;

  // With side mount
  public static final double PHONE_TILT_DEG_VUFORIA = 5.7;
  public static final double PHONE_TILT_DEG_TENSOR_FLOW = 14.0;

  private Servo phoneServo;
  double servoPosition;

  private ServoControllerEx controller;
  private int               port;

  public void init(HardwareMap hardwareMap){
    phoneServo = hardwareMap.get(Servo.class, "Servo3");
    controller = (ServoControllerEx) phoneServo.getController();
    port = phoneServo.getPortNumber();

    controller.setServoPwmEnable(port);
    setPhoneTilt(PHONE_UP_POS);
  }

  public void setPhoneTiltPos(double pos){
    servoPosition = pos;
    phoneServo.setPosition(servoPosition);
  }

  public void setPhoneTilt(double angleDeg){
    servoPosition = 0.4865 - (angleDeg/185.0);
    phoneServo.setPosition(servoPosition);
  }

  public void update(Telemetry telemetry){
    telemetry.addData("PhoneTilt", "Position %.2f", servoPosition);
  }

  public void stop(){
    controller.setServoPwmDisable(port);
  }
}
