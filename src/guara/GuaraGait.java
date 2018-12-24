package guara;

public abstract class GuaraGait
{
   /*
    * Interface to gaits;
    */
   int strokeColumns, strokeColumnsWith4Feet, flightColumns, totalOfColumns, setPointsPerColumn, totalOfPoints, sticksForOneStrokePitch;
   int gaitMatrix[][];
   double[][] pawXYZ;

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

   public double[][] getPawXYZ()
   {
      return pawXYZ;
   }
}
