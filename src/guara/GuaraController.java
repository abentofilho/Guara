package guara;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Formatter;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class GuaraController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("guaraController");

   private GuaraRobot robot;
   private GuaraKinematics kinematics;
   public GuaraWaveGait waveGait;

   private final YoInteger tickCounter = new YoInteger("tickCounter", registry);
   private final YoInteger ticksForDesiredForce = new YoInteger("ticksForDesiredForce", registry);

   private final PIDController hipAbductionController = new PIDController("hipAbductionController", registry);
   //   private final PIDController hipFlexionController = new PIDController("hipFlexionController", registry);
   //   private final PIDController kneeFlexionController = new PIDController("kneeFlexionController", registry);
   //   private final PIDController ankleFlexionController = new PIDController("ankleFlexionController", registry);

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

   public double[] thetaLeg0 = {0.0, 0.0, 0.0, 0.0};
   public double[] thetaLeg1 = {0.0, 0.0, 0.0, 0.0};
   public double[] thetaLeg2 = {0.0, 0.0, 0.0, 0.0};
   public double[] thetaLeg3 = {0.0, 0.0, 0.0, 0.0};

   //joint's angle for grap
   private YoDouble theta01, theta02, theta03, theta04, theta11, theta12, theta13, theta14, theta21, theta22, theta23, theta24, theta31, theta32, theta33,
         theta34;

   int[] pawState = {0, 0, 0, 0};
   double pawXYZ[][] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
   int setPointCounter = 0; //tick's counter for gait matrix adreeing
   int ticksForIncrementDeltaX = 128; //see velocity calculations

   int ticks = 0;
   double dt;
   Formatter fmt;
   File fileOfThetas = new File("/Users/antoniobentofilho/Dropbox/ProjetoDePesquisa/IC-IHMC/debugThetas.txt");
   BufferedWriter bw;
   FileWriter fw;

   public GuaraController(GuaraRobot robot, double dt)
   {
      this.robot = robot;
      this.dt = dt;

      ticksForDesiredForce.set(10);
      tickCounter.set(ticksForDesiredForce.getIntegerValue() + 1);

      initializeYoDoubleJointVariables(robot);

      waveGait = new GuaraWaveGait(pawState, setPointCounter);
      kinematics = new GuaraKinematics();
      assert waveGait != null;
      assert kinematics != null;
      pawXYZ = waveGait.pawXYZ;

      thetaDebugFileSetting();

      initControl();

      initializeYoDoubleJointThetas();

   }

   public void doControl()
   {
      ticks++;
      setPointCounter++; //SP counter increment
      setPointCounter = setPointCounter == waveGait.totalOfColumns ? 0 : setPointCounter;
      waveGait.getFootState(waveGait.waveGaitMatrix, pawState, setPointCounter);

      if (tickCounter.getIntegerValue() > ticksForDesiredForce.getIntegerValue())
      {
         double[] xyzPaw0, xyzPaw1, xyzPaw2, xyzPaw3;
         xyzPaw0 = waveGait.footPath(0, setPointCounter, pawXYZ[0][0], pawXYZ[0][1], pawXYZ[0][2], pawState[0]);
         xyzPaw1 = waveGait.footPath(1, setPointCounter, pawXYZ[1][0], pawXYZ[1][1], pawXYZ[1][2], pawState[1]);
         xyzPaw2 = waveGait.footPath(2, setPointCounter, pawXYZ[2][0], pawXYZ[2][1], pawXYZ[2][2], pawState[2]);
         xyzPaw3 = waveGait.footPath(3, setPointCounter, pawXYZ[3][0], pawXYZ[3][1], pawXYZ[3][2], pawState[3]);
         thetaLeg0 = kinematics.inverseKinematics(xyzPaw0);
         thetaLeg1 = kinematics.inverseKinematics(xyzPaw1);
         thetaLeg2 = kinematics.inverseKinematics(xyzPaw2);
         thetaLeg3 = kinematics.inverseKinematics(xyzPaw3);

         saveToDebugTheta(1);
         tickCounter.set(0);
      }
      tickCounter.increment();

      //loop paws

      double qAbdHip0 = robot.getAbdFlexHip0().getFirstJoint().getQ();
      double qFlexHip0 = robot.getAbdFlexHip0().getSecondJoint().getQ();
      double qFlexKnee0 = robot.getFlexKnee0().getQ();
      double qFlexAnkle0 = robot.getFlexAnkle0().getQ();
      double qAbdHip1 = robot.getAbdFlexHip1().getFirstJoint().getQ();
      double qFlexHip1 = robot.getAbdFlexHip1().getSecondJoint().getQ();
      double qFlexKnee1 = robot.getFlexKnee1().getQ();
      double qFlexAnkle1 = robot.getFlexAnkle1().getQ();
      double qAbdHip2 = robot.getAbdFlexHip2().getFirstJoint().getQ();
      double qFlexHip2 = robot.getAbdFlexHip2().getSecondJoint().getQ();
      double qFlexKnee2 = robot.getFlexKnee2().getQ();
      double qFlexAnkle2 = robot.getFlexAnkle2().getQ();
      double qAbdHip3 = robot.getAbdFlexHip3().getFirstJoint().getQ();
      double qFlexHip3 = robot.getAbdFlexHip3().getSecondJoint().getQ();
      double qFlexKnee3 = robot.getFlexKnee3().getQ();
      double qFlexAnkle3 = robot.getFlexAnkle3().getQ();

      double x0 = hipAbductionController.compute(q_abdHip0.getDoubleValue(), thetaLeg0[0], qd_abdHip0.getDoubleValue(), 0.0, dt);
      double x1 = hipAbductionController.compute(q_abdHip1.getDoubleValue(), thetaLeg0[1], qd_abdHip1.getDoubleValue(), 0.0, dt);
      double x2 = hipAbductionController.compute(q_abdHip2.getDoubleValue(), thetaLeg0[2], qd_abdHip2.getDoubleValue(), 0.0, dt);
      double x3 = hipAbductionController.compute(q_abdHip3.getDoubleValue(), thetaLeg0[3], qd_abdHip3.getDoubleValue(), 0.0, dt);
   }

   /**
    *
    */
   public void initializeYoDoubleJointThetas()
   {
      theta01 = new YoDouble("theta01", registry);
      theta02 = new YoDouble("theta02", registry);
      theta03 = new YoDouble("theta03", registry);
      theta04 = new YoDouble("theta04", registry);
      theta11 = new YoDouble("theta11", registry);
      theta12 = new YoDouble("theta12", registry);
      theta13 = new YoDouble("theta13", registry);
      theta14 = new YoDouble("theta14", registry);
      theta21 = new YoDouble("theta21", registry);
      theta22 = new YoDouble("theta22", registry);
      theta23 = new YoDouble("theta23", registry);
      theta24 = new YoDouble("theta24", registry);
      theta31 = new YoDouble("theta31", registry);
      theta32 = new YoDouble("theta32", registry);
      theta33 = new YoDouble("theta33", registry);
      theta34 = new YoDouble("theta34", registry);
   }

   /**
    * @param robot
    */
   public void initializeYoDoubleJointVariables(GuaraRobot robot)
   {
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
   }

   /**
    *
    */
   public void thetaDebugFileSetting()
   {
      /*
       * begin: thetaLeg0 debug file initializations
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
      /* end: thetaLeg0 debug file initializations */
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
         bw.write(Double.toString(pawNumber) + "\t" + Double.toString(thetaLeg0[1]) + "\t" + Double.toString(q_flexHip1.getValueAsDouble()) + "\t"
               + Double.toString(thetaLeg0[2]) + "\t" + Double.toString(q_flexHip1.getValueAsDouble()) + "\t" + Double.toString(thetaLeg0[3]) + "\t"
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
