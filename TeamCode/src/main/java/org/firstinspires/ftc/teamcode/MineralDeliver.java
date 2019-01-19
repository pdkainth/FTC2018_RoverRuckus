package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MineralDeliver extends DcMotorUpDn {

  private Servo mineralServo;
  double servoPosition;

  public void init(HardwareMap hardwareMap){
    super.init(hardwareMap, "MineralMotor", "Motor5", "Touch1");
    mineralServo = hardwareMap.get(Servo.class, "Servo2");
    collect();
  }

  public void upDn(double power){
    super.lift(power);
  }

  public void bringDown() {
    super.liftToPosition(0);
  }

  public void dump(){
    servoPosition = 1.0;
    mineralServo.setPosition(servoPosition);
  }

  public void collect(){
    servoPosition = 0.25;
    mineralServo.setPosition(servoPosition);
  }
  public void update(Telemetry telemetry){
    super.update(telemetry);
    telemetry.addData("MineralServo", "Position %.2f", servoPosition);
  }
}
