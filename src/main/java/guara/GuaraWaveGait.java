package guara;

public class GuaraWaveGait extends GuaraGait
{

   /*
    * Position controlled foothold regular and symmetric gait is , same load
    * factor for all feet and two corresponding opposite sides feet will be 180
    * degrees, or 0.5 * T out of phase. Foot flight path is a cycloid.
    */

   GuaraKinematics cin = new GuaraKinematics();
   int setPointsPerColumn = 0;
   // test variables
   double cycloidRadius = 0.1; // flight cycloid radius
   double hRobo = 0.3; //yet fixed

   int waveGaitMatrix[][] = new int[][] {

         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1},
         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
         {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

   public GuaraWaveGait(int setPoints)
   {
      // gait matrix data
      name = "WaveGait";
      strokeColumns = 27;
      strokeColumnsWith4Feet = 3;
      flightColumns = 5;
      totalOfColumns = 32;
      setPointsPerColumn = setPoints;
   }

   public void setPointsPerColumn(int setPoints)
   {
      setPointsPerColumn = setPoints;
   }

   double[] footPath(int footNumber, int setPointCounter, double x4, double y4, double z4)
   {
      //		System.out.println("GuaraWaveGait");
      double[] xyz = {0.0, 0.0, 0.0};
      int columnCounter = (int) (setPointCounter / (totalOfColumns * setPointsPerColumn)); // for matrix set point column  counter
      int footState = waveGaitMatrix[footNumber][(int) (setPointCounter - columnCounter * totalOfColumns) - 1];
      switch (footState)
      {
      case 0: // foot in flight
         xyz[0] = x4 + DeltaX();
         xyz[1] = y4;
         xyz[2] = z4;
         break;
      case 1: /*
               * foot in stroke; straight walk without lateral movement and the
               * support phase is a straight trajectory parallel to the robot's
               * body
               */
         double deltaX = DeltaX();
         double teta = deltaX / cycloidRadius;

         xyz[0] = x4 + DeltaX();
         xyz[1] = y4;
         xyz[2] = z4 + (1 - Math.cos(teta)) * cycloidRadius;
         break;
      }
      return xyz;
   }

   double[] legJoints(int legNumber, int setPointCounter, double x4, double y4, double z4)
   {
      double[] xyz = footPath(legNumber, setPointCounter, x4, y4, z4);
      double[] joints = cin.cinInv(xyz);
      return joints;
   }
}
