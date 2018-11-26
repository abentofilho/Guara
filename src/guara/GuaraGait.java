package guara;

public abstract class GuaraGait
{
   /*
    * Interface to gaits;
    */
   int strokeColumns, strokeColumnsWith4Feet, flightColumns, totalOfColumns, setPointsPerColumn,totalOfPoints,sticksForOneStrokePitch;
   int gaitMatrix[][];
   int[] vetorPatas = new int[4];
   int setPointCounter, columnsCounter;
   double deltaX, strokePitch, robotHeight, velocity;
   double a2, a3, a4;

   String name;

   // testing variables

   public GuaraGait()
   {
      // TODO Auto-generated constructor stub
   }

   double getCiclosDeApoio()
   {
      return strokeColumns;
   };

   double getCiclosDeApoioCom4patas()
   {
      return strokeColumnsWith4Feet;
   }

   double getCiclosDeVo()
   {
      return flightColumns;
   }

   double TotalDeCiclos()
   {
      return totalOfColumns;
   }

   String getName()
   {
      return name;
   }

   void setSPCouter(int spCounter)
   {
      setPointCounter = spCounter;
   }



   //   double[] trajPata(int iPata, int iSetPoint, double x4, double y4, double z4, double delta)
   //   {
   //      return xyz;
   //   };

   // double[] trajVoo(double[] vetorPatas, double x4, double y4, double z4,
   // double velRobo, double delta) {
   // return xyz;
   // }

   // double[] getVetorPatas(int iCiclo) {
   // //
   // int i = (int) (iCiclo > 16 ? iCiclo - (iCiclo - 1) * totalDeCiclos
   // : iCiclo);
   // //
   // vetorPatas[0] = waveGaitMatrix[i]; // pata0
   // vetorPatas[1] = waveGaitMatrix[(int) (i + totalDeCiclos)]; // pata1
   // vetorPatas[2] = waveGaitMatrix[(int) (i + 2 * totalDeCiclos)];// pata2
   // vetorPatas[3] = waveGaitMatrix[(int) (i + 3 * totalDeCiclos)];// pata3
   // //
   // return vetorPatas;
   // }

}
