package guara;

public class GuaraKinematics //extends GuaraRobot
{

   GuaraRobot rob;
   GuaraGait gait;

   // Variables

   double erro = 1.0e-15; // Trigonometric functions threshold
   double[] anklePositionVector; // ankle's position vector in operational RF
   double[] theta3; // joint 3 (ankle) joint space coordinates vector
   double a2, a3, a4; // leg's links lengths

   public GuaraKinematics()
   {
   }

   public GuaraKinematics(GuaraRobot robot, GuaraGait gait)
   {
      //		rob = new GuaraRobot();
      //
      this.rob = robot;
      this.gait = gait;
      a2 = rob.a2();
      a3 = rob.a3();
      a4 = rob.a4();

   }

   // methods definitions

   public double[][] DH(double a, double d, double alfa, double teta)
   {

      /*
       * homogeneous transform matrices between link's reference frames
       * according to Denavit - Hartenberg convention These transformations are
       * implemented in SCS;
       */
      double cosTeta = (Math.cos(teta) > erro || Math.cos(teta) < -erro) ? Math.cos(teta) : 0.0;
      double sinTeta = (Math.sin(teta) > erro || Math.sin(teta) < -erro) ? Math.sin(teta) : 0.0;
      double cosAlfa = (Math.cos(alfa) > erro || Math.cos(alfa) < -erro) ? Math.cos(alfa) : 0.0;
      double sinAlfa = (Math.sin(alfa) > erro || Math.sin(alfa) < -erro) ? Math.sin(alfa) : 0.0;

      double A[][] = new double[4][4];
      for (int i = 0; i < 4; i++)
         for (int j = 0; j < 4; j++)
            A[i][j] = 0.0;

      A[0][0] = cosTeta;
      A[0][1] = -sinTeta * cosAlfa;
      A[0][2] = sinTeta * sinAlfa;
      A[0][3] = a * cosTeta;

      A[1][0] = sinTeta;
      A[1][1] = cosTeta * cosAlfa;
      A[1][2] = -cosTeta * sinAlfa;
      A[1][3] = a * sinTeta;

      A[2][0] = 0;
      A[2][1] = sinAlfa;
      A[2][2] = cosAlfa;
      A[2][3] = d;

      A[3][0] = 0;
      A[3][1] = 0;
      A[3][2] = 0;
      A[3][3] = 1;

      return A;

   }

   // Inverse kinematics: joint 3 (ankle) variables as functions of paw position [x4,y4,z4]

   public double[] inverseKinematics(double x4, double y4, double z4, double a4, double theta1, double theta4)
   {
      /*
       * Inverse kinematics from joint 4 (paw) to joint 3 (ankle).
       */

      anklePositionVector = new double[] {0.0, 0.0, 0.0};

      anklePositionVector[0] = x4
            - (double) ((a4 * Math.cos(theta4) * Math.sin(theta1) > erro || a4 * Math.cos(theta4) * Math.sin(theta1) < -erro) ? a4 * Math.sin(theta4) : 0); // x3

      anklePositionVector[1] = y4
            - (double) ((a4 * Math.cos(theta4) * Math.cos(theta1) > erro || a4 * Math.cos(theta4) * Math.cos(theta1) < -erro) ? a4 * Math.sin(theta4) : 0); // y3

      anklePositionVector[2] = z4 - (double) ((a4 * Math.sin(theta4) > erro || a4 * Math.sin(theta4) < -erro) ? a4 * Math.sin(theta4) : 0); // z3;
      return anklePositionVector;
   }

   public void inverseKinematics(int pawNumber, double[][] thetaLegToPack, GuaraGait gait)
   {

      /*
       * Inverse kinematics is done for joint 3 (ankle). From paw's coordinates
       * [x4,y4,z4] we get joint 3 [x3,y3,z3] by locally defined kinematics and
       * look for joint angles.
       */

      double theta[] = {0.0, 0.0, 0.0, 0.0};
      double cosTheta = 0.0, sinTeta = 0.0;

      double[][] pawCoordinates = gait.getPawXYZ(); //gait is defined in GuaraController class
      double x3 = pawCoordinates[pawNumber][0];
      double y3 = pawCoordinates[pawNumber][1];
      double z3 = pawCoordinates[pawNumber][2];

      theta[0] = Math.atan2(y3, x3);
      // cosTeta = (Math.pow(a2, 2) + Math.pow(waveGait, 2) - Math.pow(y3, 2) - Math
      cosTheta = (Math.pow(x3, 2) + Math.pow(y3, 2) + Math.pow(z3, 2) - Math.pow(a2, 2) - Math.pow(a3, 2)) / (2 * a2 * a3);
      sinTeta = cosTheta == 0 ? 1.0 : Math.sqrt((1 - Math.pow(cosTheta, 2)));
      theta[2] = Math.atan2(sinTeta, cosTheta);
      double alfa = Math.atan2(a3 * sinTeta, (a2 + a3 * cosTheta));
      double beta = Math.atan2(Math.sqrt(Math.pow(y3, 2) + Math.pow(x3, 2)), Math.sqrt(Math.pow(x3, 2) + Math.pow(z3, 2)));
      theta[1] = (Math.abs(z3) > Math.abs(y3)) ? beta + alfa : beta - alfa;

      //theta[3] = theta[2] - Math.PI / 6.0;// set in robot class
      double [] ankleTheta=rob.getFlexAnkleAngle();
      thetaLegToPack[pawNumber][0] = theta[0];
      thetaLegToPack[pawNumber][1] = theta[1];
      thetaLegToPack[pawNumber][2] = theta[2];
      thetaLegToPack[pawNumber][3] = ankleTheta[pawNumber];//theta[3];
   }

   public double[][] MatrixMultiplication(double[][] m, double[][] n)
   {
      double c[][] = new double[4][4];
      double sum;
      for (int i = 0; i < 4; i++)
      {
         for (int j = 0; j < 4; j++)
         {
            sum = 0;
            for (int k = 0; k < 4; k++)
            {
               sum = sum + (m[i][k] * n[k][j]);
               c[i][j] = sum;
            }
         }
      }
      return c;
   }

}
