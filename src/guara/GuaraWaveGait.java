package guara;

public class GuaraWaveGait extends GuaraGait
{

   /*
    * Position controlled foothold regular and symmetric gait, same load factor
    * for all feet and two corresponding opposite sides feet will be 180
    * degrees, or 0.5 * T out of phase. Foot flight path is a polygonal.
    */

//   GuaraKinematics cin = new GuaraKinematics();
   //   int setPointsPerColumn = 4; //see documentation used in Guara robot
   //   double velocity = 1.0 / 7.0 * 1000 / 3600; // man's typical runs 1 km in 7 min leading to a speed of 0.04 m/s
   //   double strokePitch = 0.192; //m distance traveled by tobot's body under a full foot stroke.
   //   double delta = 0.192 / setPointsPerColumn/totalOfColumns; //mm the increment/100 ticks to which we'll run the robot to 0.04 m/s
   //   double hRobo = 0.277; //see documentation

   //1 tick - 0.0004 s
   //x tick - 1.0 s x = 2500 tick/s
   //0.04 m/s = 0,04m/(tick/2500) = 1  m/tick
   //1m - 1tick; 0.04 - x -> x = 2500 tick
   //strideTime = 0.192m/0.04m/s = 4.8 s equivalent to 12000 ticks
   //delta = 0.192/12000-.delta=16 micromilimiters; that's no reasonable!
   //let's adopt 108 increments in stroke and 20 in flight, 4 ticks per gait matrix column,
   // leading to a delta = 0.192/128 = 0.0015 m = 1.5 mm

   // test variables
   double cycloidRadius = 0.1; // flight cycloid radius
   int[][] waveGaitMatrix = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
         {1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
   int [] footStateToPack;

   public GuaraWaveGait(int[] feetStateToPack, int setPointCouner)
   {
      // gait matrix data
      name = "WaveGait";
      strokeColumns = 27;
      strokeColumnsWith4Feet = 3;
      flightColumns = 5;
      totalOfColumns = 32;
      feetStateToPack = new int[4];
      pawXYZ = new double[4][3];
      setPointsPerColumn = 4; //see documentation; used in Guara robot
      velocity = 1.0 / 7.0 * 1000 / 3600; // man's typical runs 1 km in 7 min leading to a speed of 0.04 m/s
      strokePitch = 0.192; //m distance traveled by robot's body under a full foot stroke.
      deltaX = 0.192 / setPointsPerColumn / totalOfColumns; //mm the increment/100 ticks to which we'll run the robot to 0.04 m/s
      robotHeight = 0.277; //see documentation
      sticksForOneStrokePitch = setPointsPerColumn * totalOfColumns;
      getFootState(waveGaitMatrix, feetStateToPack, setPointCounter);
//      waveGaitMatrix = data;
      //paw coordinates in leg's RF
      pawXYZ[0][0] = 0.0;
      pawXYZ[0][1] = 0.0;
      pawXYZ[0][2] = -robotHeight;
      pawXYZ[1][0] = 0.0;
      pawXYZ[1][1] = 0.0;
      pawXYZ[1][2] = -robotHeight;
      pawXYZ[2][0] = 0.0;
      pawXYZ[2][1] = 0.0;
      pawXYZ[2][2] = -robotHeight;
      pawXYZ[3][0] = 0.0;
      pawXYZ[3][1] = 0.0;
      pawXYZ[3][2] = -robotHeight;
   }

   void footPath(int pawNumber, int setPointCounter, int pawState)
   {
      switch (pawState)
      {
      case 0: /*
               * foot in flight is a cycloid
               */
         double teta = deltaX / cycloidRadius;
         pawXYZ[pawNumber][0] += deltaX;
         //pawXYZ[pawNumber][1] += deltaY; body side move not yet implemented
         pawXYZ[pawNumber][2] += (1 - Math.cos(teta)) * cycloidRadius;
         break;
      case 1: /*
               * foot in stroke; straight walk without lateral movement and the
               * support phase is a straight trajectory parallel to the robot's
               * body
               */
         pawXYZ[pawNumber][0] += deltaX;
         //pawXYZ[pawNumber][1] += deltaY;   body side move not yet implemented
         //pawXYZ[pawNumber][2] += deltaZ;   body up-down move not yet implemented
         break;
      }
   }

 /*  double[] legJoints(int legNumber, int setPointCounter, double x4, double y4, double z4, int footState)
   {
      //double[] footPath(int pawNumber, int setPointCounter, double x4, double y4, double z4, int footState)
      footPath(legNumber, setPointCounter, footState);
      double[] joints = cin.inverseKinematics(legNumber);
      return joints;
   }
*/


   void getFootState(int[][] GaitMatrix, int [] feetStateToPack, int setPointCounter)
   {
      // TODO Auto-generated method stub
        columnsCounter = setPointCounter / setPointsPerColumn;
      feetStateToPack[0] = GaitMatrix[0][columnsCounter];
      feetStateToPack[1] = GaitMatrix[1][columnsCounter];
      feetStateToPack[2] = GaitMatrix[2][columnsCounter];
      feetStateToPack[3] = GaitMatrix[3][columnsCounter];

   }

   public int[][] getWaveGaitMatrix()
   {
      return waveGaitMatrix;
   }


}
