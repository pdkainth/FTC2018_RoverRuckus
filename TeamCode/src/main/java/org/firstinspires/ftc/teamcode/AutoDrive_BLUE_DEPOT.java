package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Drive : BLUE DEPOT")
public class AutoDrive_BLUE_DEPOT extends MyAutoDrive {

  @Override
  public void init() {
    setAlliance(AllianceColor.BLUE, AllianceSide.DEPOT);
    super.init();

    autoRoute.landingPos.setPos(-18.0f, 14.0f, -135.0f);
    autoRoute.mineralPos[0].setPos(-16.0f, 44.0f, 180.0f); // Right
    autoRoute.mineralPos[1].setPos(-30.0f, 22.0f, 90.0f);  // Center
    autoRoute.mineralPos[2].setPos(-38.5f, 13.5f, 90.0f);  // Left

    autoRoute.mineralMoveDistance[0] = 10;
    autoRoute.mineralMoveDistance[1] = 10;
    autoRoute.mineralMoveDistance[2] = 10;

    autoRoute.depotRoutePos[0].setPos(-40.0f, 12.0f, -135.0f);
    autoRoute.depotRoutePos[1].setPos(-60.0f, 12.0f, 90.0f);
    autoRoute.depotRoutePos[2].setPos(-60.0f, 60.0f, -90.0f);

    autoRoute.craterPos.setPos(-60.0f, -30.0f, -90.0f);

    autoRoute.vuforiaAngle = 160.0f;
  }

  public void start(){
    super.start();

  }

}
