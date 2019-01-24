package guara;

public abstract class GuaraGait
{
   /*
    * Esta e uma classe abstrata que implementa a interface , que sao os
    * metodos e variaveis abstratas e concretas das andaduras. Cada andadura
    * tera uma classe concreta para as variaveis e metodos abstratos
    * necessarios .
    */
   double strokeColumns, strokeColumnsWith4Feet, flightColumns, totalOfColumns;
   int gaitMatrix[][];
   int[] vetorPatas = new int[4];
   double[][][] xyz = new double[1][1][1];
   double deltaX;
   double a2, a3, a4;
   String name;

   // testing variables
   double DeltaX()
   {
      return deltaX = (double) ((6.0 * 1000.0 / 3600.0) * (4.0 / 10000.0)); // velocity [m/s]/DT=4/10000;
   }

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

//   double[] trajPata(int iPata, int iSetPoint, double x4, double y4, double z4, double deltaX)
//   {
//      return xyz;
//   };

   // double[] trajVoo(double[] vetorPatas, double x4, double y4, double z4,
   // double velRobo, double deltaX) {
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
