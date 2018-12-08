package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends DcMotorUpDn {

  public static final int LIFT_ENC_POS_HANG = 15000;
  public static final int LIFT_ENC_POS_MIN = 0;

  public void init(HardwareMap hardwareMap) {
    super.init(hardwareMap, "Lift", "Motor4", "Touch0");
  }
}
