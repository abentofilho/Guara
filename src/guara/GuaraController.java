package guara;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Formatter;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class GuaraController implements RobotController
{
   private GuaraRobot robot;
   private GuaraKinematics kinematics;
   public GuaraWaveGait waveGait;

   private final YoVariableRegistry registry = new YoVariableRegistry("guaraController");

   // constantes do controlador da perna

   double k1, k2, k3, k0;
   double kd1, kd2, kd3, kd0;

   private YoDouble tau_abdHip0, tau_flexHip0, tau_abdHip1, tau_flexHip1, tau_abdHip2, tau_flexHip2, tau_abdHip3, tau_flexHip3;
   private YoDouble q_abdHip0, q_flexHip0, q_abdHip1, q_flexHip1, q_abdHip2, q_flexHip2, q_abdHip3, q_flexHip3;
   private YoDouble qd_abdHip0, qd_flexHip0, qd_abdHip1, qd_flexHip1, qd_abdHip2, qd_flexHip2, qd_abdHip3, qd_flexHip3;

   private YoDouble tau_flexKnee0, tau_flexKnee1, tau_flexKnee2, tau_flexKnee3;
   private YoDouble q_flexKnee0, q_flexKnee1, q_flexKnee2, q_flexKnee3, qd_flexKnee0, qd_flexKnee1, qd_flexKnee2, qd_flexKnee3;

   private YoDouble tau_flexAnkle0, tau_flexAnkle1, tau_flexAnkle2, tau_flexAnkle3;
   private YoDouble q_flexAnkle0, q_flexAnkle1, q_flexAnkle2, q_flexAnkle3, qd_flexAnkle0, qd_flexAnkle1, qd_flexAnkle2, qd_flexAnkle3;

   // 4
   // set points counter

   int i = 0;

   public double[] theta = {0.0, 0.0, 0.0};

   //inverse kinematics
   private YoDouble theta01 = new YoDouble("theta01", registry);
   private YoDouble theta02 = new YoDouble("theta02", registry);
   private YoDouble theta03 = new YoDouble("theta03", registry);
   private YoDouble theta04 = new YoDouble("theta04", registry);
   private YoDouble theta11 = new YoDouble("theta11", registry);
   private YoDouble theta12 = new YoDouble("theta12", registry);
   private YoDouble theta13 = new YoDouble("theta13", registry);
   private YoDouble theta14 = new YoDouble("theta14", registry);
   private YoDouble theta21 = new YoDouble("theta21", registry);
   private YoDouble theta22 = new YoDouble("theta22", registry);
   private YoDouble theta23 = new YoDouble("theta23", registry);
   private YoDouble theta24 = new YoDouble("theta24", registry);
   private YoDouble theta31 = new YoDouble("theta31", registry);
   private YoDouble theta32 = new YoDouble("theta32", registry);
   private YoDouble theta33 = new YoDouble("theta33", registry);
   private YoDouble theta34 = new YoDouble("theta34", registry);

   int[] pawState = {0, 0, 0, 0};
   double pawXYZ[][] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
   int setPointCounter = 0;
   int incrementCounter = 0;
   int sticksForIncrementDeltaX = 128; //see velocity calculations

   int ticks = 0;

   Formatter fmt;
   File fileOfThetas = new File("/Users/antoniobentofilho/Dropbox/ProjetoDePesquisa/IC-IHMC/debugThetas.txt");
   BufferedWriter bw;
   FileWriter fw;

   public GuaraController(GuaraRobot robot)
   { //, String name) {

      tau_abdHip0 = (YoDouble) robot.getVariable("tau_abdHip0");
      tau_abdHip1 = (YoDouble) robot.getVariable("tau_abdHip1");
      tau_abdHip2 = (YoDouble) robot.getVariable("tau_abdHip2");
      tau_abdHip3 = (YoDouble) robot.getVariable("tau_abdHip3");
      q_abdHip0 = (YoDouble) robot.getVariable("q_abdHip0");
      q_abdHip1 = (YoDouble) robot.getVariable("q_abdHip1");
      q_abdHip2 = (YoDouble) robot.getVariable("q_abdHip2");
      q_abdHip3 = (YoDouble) robot.getVariable("q_abdHip3");
      qd_abdHip0 = (YoDouble) robot.getVariable("qd_abdHip0");
      qd_abdHip1 = (YoDouble) robot.getVariable("qd_abdHip1");
      qd_abdHip2 = (YoDouble) robot.getVariable("qd_abdHip2");
      qd_abdHip3 = (YoDouble) robot.getVariable("qd_abdHip3");

      tau_flexHip0 = (YoDouble) robot.getVariable("tau_flexHip0");
      tau_flexHip1 = (YoDouble) robot.getVariable("tau_flexHip1");
      tau_flexHip2 = (YoDouble) robot.getVariable("tau_flexHip2");
      tau_flexHip3 = (YoDouble) robot.getVariable("tau_flexHip3");
      q_flexHip0 = (YoDouble) robot.getVariable("q_flexHip0");
      q_flexHip1 = (YoDouble) robot.getVariable("q_flexHip1");
      q_flexHip2 = (YoDouble) robot.getVariable("q_flexHip2");
      q_flexHip3 = (YoDouble) robot.getVariable("q_flexHip3");
      qd_flexHip0 = (YoDouble) robot.getVariable("qd_flexHip0");
      qd_flexHip1 = (YoDouble) robot.getVariable("qd_flexHip1");
      qd_flexHip2 = (YoDouble) robot.getVariable("qd_flexHip2");
      qd_flexHip3 = (YoDouble) robot.getVariable("qd_flexHip3");

      tau_flexKnee0 = (YoDouble) robot.getVariable("tau_flexKnee0");
      tau_flexKnee1 = (YoDouble) robot.getVariable("tau_flexKnee1");
      tau_flexKnee2 = (YoDouble) robot.getVariable("tau_flexKnee2");
      tau_flexKnee3 = (YoDouble) robot.getVariable("tau_flexKnee3");
      q_flexKnee0 = (YoDouble) robot.getVariable("q_flexKnee0");
      q_flexKnee1 = (YoDouble) robot.getVariable("q_flexKnee1");
      q_flexKnee2 = (YoDouble) robot.getVariable("q_flexKnee2");
      q_flexKnee3 = (YoDouble) robot.getVariable("q_flexKnee3");
      qd_flexKnee0 = (YoDouble) robot.getVariable("qd_flexKnee0");
      qd_flexKnee1 = (YoDouble) robot.getVariable("qd_flexKnee1");
      qd_flexKnee2 = (YoDouble) robot.getVariable("qd_flexKnee2");
      qd_flexKnee3 = (YoDouble) robot.getVariable("qd_flexKnee3");

      tau_flexAnkle0 = (YoDouble) robot.getVariable("tau_flexAnkle0");
      tau_flexAnkle1 = (YoDouble) robot.getVariable("tau_flexAnkle1");
      tau_flexAnkle2 = (YoDouble) robot.getVariable("tau_flexAnkle2");
      tau_flexAnkle3 = (YoDouble) robot.getVariable("tau_flexAnkle3");
      q_flexAnkle0 = (YoDouble) robot.getVariable("q_flexAnkle0");
      q_flexAnkle1 = (YoDouble) robot.getVariable("q_flexAnkle1");
      q_flexAnkle2 = (YoDouble) robot.getVariable("q_flexAnkle2");
      q_flexAnkle3 = (YoDouble) robot.getVariable("q_flexAnkle3");
      qd_flexAnkle0 = (YoDouble) robot.getVariable("qd_flexAnkle0");
      qd_flexAnkle1 = (YoDouble) robot.getVariable("qd_flexAnkle1");
      qd_flexAnkle2 = (YoDouble) robot.getVariable("qd_flexAnkle2");
      qd_flexAnkle3 = (YoDouble) robot.getVariable("qd_flexAnkle3");

      this.robot = robot;
      waveGait = new GuaraWaveGait(pawState, setPointCounter);
      kinematics = new GuaraKinematics();
      assert waveGait != null;
      assert kinematics != null;
      pawXYZ = waveGait.pawXYZ;

      /*
       * begin: theta debug file initializations
       */
      try
      {
         //
         if (!fileOfThetas.exists())
         {
            // cria um arquivo (vazio)
            fileOfThetas.createNewFile();
         }
      }
      catch (IOException ex)
      {
         ex.printStackTrace();
      }
      try
      {
         fw = new FileWriter(fileOfThetas, true);
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      assert (fw != null);
      bw = new BufferedWriter(fw);
      assert (bw != null);
      //      fmt.format("Leg \t Theta0 \t Theta1 \t Theta2 \t Theta3");
      //      System.out.println(fmt);
      /* end: theta debug file initializations */

      initControl();
      System.out.println("saiu initcontrol");

   }

   public void initControl()
   {

      k0 = 3;//10;//300;
      k1 = 2;//20;//250;
      k2 = 3;//30;//300;
      k3 = 4;//2;//1;//-150;
      kd0 = 0.3;//1;//3;
      kd1 = 0.3;//3;
      kd2 = 0.5;//1;//5;
      kd3 = 0.5;//2;//1;//5;

   }

   public void doControl()
   {
      ticks++;
      setPointCounter++; //SP counter increment
      setPointCounter = setPointCounter == waveGait.totalOfColumns ? 0 : setPointCounter;
      waveGait.getFootState(waveGait.waveGaitMatrix, pawState, setPointCounter);

      //loop paws

      for (int pawNumber = 0; pawNumber < 4; pawNumber++)
      {
         double[] temp;
         temp = waveGait.footPath(pawNumber, setPointCounter, pawXYZ[pawNumber][0], pawXYZ[pawNumber][1], pawXYZ[pawNumber][2], pawState[i]);

         theta = kinematics.inverseKinematics(temp);//pawXYZ[pawNumber]);

         saveToDebugTheta(pawNumber);

         switch (pawNumber)
         {
         case 0:
            tau_abdHip0.set(k0 * (theta[0] - q_abdHip0.getValueAsDouble()) + kd0 * (0 - qd_abdHip0.getValueAsDouble()));
            tau_flexHip0.set(k1 * (theta[1] + q_flexHip0.getValueAsDouble()) + kd1 * (0 - qd_flexHip0.getValueAsDouble()));
            tau_flexKnee0.set(k2 * (theta[2] - q_flexKnee0.getValueAsDouble()) + kd2 * (0 - qd_flexKnee0.getValueAsDouble()));
            tau_flexAnkle0.set(k3 * (theta[3] - q_flexAnkle0.getValueAsDouble()) + kd3 * (0 - qd_flexAnkle0.getValueAsDouble()));
            theta01.set(theta[0]);
            theta02.set(theta[1]);
            theta03.set(theta[2]);
            theta04.set(theta[3]);
            break;
         case 1:
            tau_abdHip1.set(k0 * (theta[0] - q_abdHip1.getValueAsDouble()) + kd0 * (0 - qd_abdHip1.getValueAsDouble()));
            tau_flexHip1.set(k1 * (theta[1] + q_flexHip1.getValueAsDouble()) + kd1 * (0 - qd_flexHip1.getValueAsDouble()));
            tau_flexKnee1.set(k2 * (theta[2] - q_flexKnee1.getValueAsDouble()) + kd2 * (0 - qd_flexKnee1.getValueAsDouble()));
            tau_flexAnkle1.set(k3 * (theta[3] - q_flexAnkle1.getValueAsDouble()) + kd3 * (0 - qd_flexAnkle1.getValueAsDouble()));
            theta11.set(theta[0]);
            theta12.set(theta[1]);
            theta13.set(theta[2]);
            theta14.set(theta[3]);
            break;
         case 2:
            tau_abdHip2.set(k0 * (theta[0] - q_abdHip2.getValueAsDouble()) + kd0 * (0 - qd_abdHip2.getValueAsDouble()));
            tau_flexHip2.set(k1 * (theta[1] + q_flexHip2.getValueAsDouble()) + kd1 * (0 - qd_flexHip2.getValueAsDouble()));
            tau_flexKnee2.set(k2 * (theta[2] - q_flexKnee2.getValueAsDouble()) + kd2 * (0 - qd_flexKnee2.getValueAsDouble()));
            tau_flexAnkle2.set(k3 * (theta[3] - q_flexAnkle2.getValueAsDouble()) + kd3 * (0 - qd_flexAnkle2.getValueAsDouble()));
            theta21.set(theta[0]);
            theta22.set(theta[1]);
            theta23.set(theta[2]);
            theta24.set(theta[3]);
            break;
         case 3:
            tau_abdHip3.set(k0 * (theta[0] - q_abdHip3.getValueAsDouble()) + kd0 * (0 - qd_abdHip3.getValueAsDouble()));
            tau_flexHip3.set(k1 * (theta[1] + q_flexHip3.getValueAsDouble()) + kd1 * (0 - qd_flexHip3.getValueAsDouble()));
            tau_flexKnee3.set(k2 * (theta[2] - q_flexKnee3.getValueAsDouble()) + kd2 * (0 - qd_flexKnee3.getValueAsDouble()));
            tau_flexAnkle3.set(k3 * (theta[3] - q_flexAnkle3.getValueAsDouble()) + kd3 * (0 - qd_flexAnkle3.getValueAsDouble()));
            theta31.set(theta[0]);
            theta32.set(theta[1]);
            theta33.set(theta[2]);
            theta34.set(theta[3]);
            break;
         default:
            pawNumber = 9;
         }

      }
   }

   /**
    * @param pawNumber
    */
   public void saveToDebugTheta(int pawNumber)
   {
      try
      {
         saveTheta(pawNumber);
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   /**
    * @param pawNumber
    * @throws IOException
    */
   public void saveTheta(int pawNumber) throws IOException
   {
      if (ticks <= 1000)
      {
         bw.write(Double.toString(pawNumber) + "\t" + Double.toString(theta[1]) + "\t" + Double.toString(q_flexHip1.getValueAsDouble()) + "\t"
               + Double.toString(theta[2]) + "\t" + Double.toString(q_flexHip1.getValueAsDouble()) +"\t"+ Double.toString(theta[3])+"\t"
               + Double.toString(q_flexHip3.getValueAsDouble()));
         bw.newLine();
      }
      else
      {
         bw.close();
         fw.close();
      }
   }

   public String getDescription()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public String getName()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      // TODO Auto-generated method stub
      return registry;
   }

   public void initialize()
   {
      // TODO Auto-generated method stub

   }

}
