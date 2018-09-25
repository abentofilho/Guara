package guara;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
//import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.*;

public class GuaraSimulation
{
   SimulationConstructionSet sim;
   Thread myThread;
   double robotSpeed = 6 * 1000 / 3600; // 6 km/h
   public static final double TIME = 20.0;
   private final GuaraRobot guara;

   public GuaraSimulation(){

      guara = new GuaraRobot();
      sim = new SimulationConstructionSet(guara);
      GuaraController guaraController = new GuaraController(guara);
      guara.setController(guaraController);
      		System.out.println("Guara.setController------------------------------");
      sim.setGroundVisible(true);
      sim.setCameraTracking(false, false, false, false);
      sim.setCameraDolly(false, false, false, false);
      sim.setCameraPosition(7.7, 4.5, 3.0);
      sim.setCameraFix(0.0, 0.0, 0.8);
      // sim.setCameraTrackingVars("ef_track00_x", "ef_track00_y",
      // "ef_track00_z");

      sim.setDT(0.0004, 10); //
      myThread = new Thread(sim);
      myThread.start();
   }

//   public boolean run(double simulationTime) throws SimulationExceededMaximumTimeException
//   {
//      return blockingSimulationRunner.simulateAndBlockAndCatchExceptions(simulationTime);
//   }

   public static void main(String[] args) throws SimulationExceededMaximumTimeException
   {
      GuaraSimulation guaraSimulation = new GuaraSimulation();
//      guaraSimulation.run(TIME);
////      ThreadTools.sleepForever();
   }

}
