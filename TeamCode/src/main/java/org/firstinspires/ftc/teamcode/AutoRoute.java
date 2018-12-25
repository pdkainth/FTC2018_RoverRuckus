package org.firstinspires.ftc.teamcode;

public class AutoRoute {

  public static final int NUM_MINERAL_POS = 3;
  public static final int NUM_DEPOT_ROUTE_POS = 3;


  Position landingPos = new Position();
  Position[] mineralPos = new Position[NUM_MINERAL_POS];
  float[] mineralMoveDistance = new float[NUM_MINERAL_POS];
  float[] mineralMoveBackDistance = new float[NUM_MINERAL_POS];
  int[] numDepotRoutePos = {0, 0, 0};
  Position[][] depotRoutePos = new Position[NUM_MINERAL_POS][NUM_DEPOT_ROUTE_POS];
  Position craterPos = new Position();

  float preVuforiaDrive;
  float preVuforiaAngle;
  float vuforiaAngle;

  public AutoRoute()
  {
    for (int index = 0; index < NUM_MINERAL_POS; index++){
      mineralPos[index] = new Position();
    }
    for (int index = 0; index < NUM_DEPOT_ROUTE_POS; index++){
      for (int index1 = 0; index1 < NUM_DEPOT_ROUTE_POS; index1++) {
        depotRoutePos[index][index1] = new Position();
      }
    }
  }

  public void setDepotRoutePos() {
    landingPos.setPos(-18.0f, 14.0f, -135.0f);
    mineralPos[0].setPos(-18.0f, 44.0f, 160.0f); // Right
    mineralPos[1].setPos(-26.0f, 26.0f, 135.0f);  // Center
    mineralPos[2].setPos(-46.0f, 12.0f, 90.0f);  // Left

    mineralMoveDistance[0] = 10.0f;
    mineralMoveDistance[1] = 10.0f;
    mineralMoveDistance[2] = 10.0f;

    mineralMoveBackDistance[0] = 0.0f;
    mineralMoveBackDistance[1] = 0.0f;
    mineralMoveBackDistance[2] = 0.0f;

    numDepotRoutePos[0] = 1;
    depotRoutePos[0][0].setPos(-54.0f, 50.0f, -45.0f);

    numDepotRoutePos[1] = 1;
    depotRoutePos[1][0].setPos(-54.0f, 42.0f, -45.0f);

    numDepotRoutePos[2] = 1;
    depotRoutePos[2][0].setPos(-54.0f, 42.0f, -45.0f);

    craterPos.setPos(-60.0f, -30.0f, -90.0f);

    preVuforiaDrive = 0.0f;
    preVuforiaAngle = 160.0f;
    vuforiaAngle = 160.0f;
  }

  public void setCraterRoutePos() {
    landingPos.setPos(14.0f, 18.0f, 135.0f);
    mineralPos[2].setPos(18.0f, 42.0f, 45.0f); // Right
    mineralPos[1].setPos(27.0f, 27.0f, 45.0f);  // Center
    mineralPos[0].setPos(40.0f, 20.0f, 45.0f);  // Left

    mineralMoveDistance[0] = 20.0f;
    mineralMoveDistance[1] = 20.0f;
    mineralMoveDistance[2] = 20.0f;

    mineralMoveBackDistance[0] = 0.0f;
    mineralMoveBackDistance[1] = 0.0f;
    mineralMoveBackDistance[2] = 0.0f;

    numDepotRoutePos[0] = 0;
    //depotRoutePos[0][0].setPos(6.0f, 50.0f, 180.0f);
    //depotRoutePos[0][1].setPos(60.0f, 12.0f, 90.0f);
    //depotRoutePos[0][1].setPos(-54.0f, 54.0f, -45.0f);

    numDepotRoutePos[1] = 0;
    //depotRoutePos[1][0].setPos(6.0f, 48.0f, 135.0f);
    //depotRoutePos[1][1].setPos(60.0f, 12.0f, 90.0f);
    //depotRoutePos[1][2].setPos(-60.0f, 60.0f, -45.0f);

    numDepotRoutePos[2] = 0;
    //depotRoutePos[2][0].setPos(6.0f, 48.0f, 135.0f);
    //depotRoutePos[2][1].setPos(60.0f, 12.0f, 90.0f);
    //depotRoutePos[2][2].setPos(-60.0f, 60.0f, -45.0f);


    //craterPos.setPos(-60.0f, -30.0f, -90.0f);

    preVuforiaDrive = 6.0f;
    preVuforiaAngle = 90.0f;
    vuforiaAngle = -170.0f;
  }

}

