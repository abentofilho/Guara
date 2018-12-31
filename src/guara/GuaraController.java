package guara;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Formatter;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class GuaraController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("guaraController");

   private GuaraKinematics kinematics;
   public GuaraWaveGait waveGait;
   public GuaraRobot robot;
   private final YoInteger tickCounter = new YoInteger("tickCounter", registry);
   private final YoInteger ticksForDesiredForce = new YoInteger("ticksForDesiredForce", registry);

   private final PIDController tauFlexAnkleController = new PIDController("tauFlexAnkleController", registry);
   private final PIDController tauAbdHipController = new PIDController("tauAbdHipController", registry);
   private final PIDController tauAbduHipController = new PIDController("tauAbduHipController", registry);

   private final PIDController tauFlexHipController = new PIDController("tauFlexHipController", registry);
   private final PIDController tauFlexKneeController = new PIDController("tauFlexKneeController", registry);

   /*
    * leg controller proportional constants
    */
   YoDouble kpAbduHip0, kpAbduHip1, kpAbduHip2, kpAbduHip3;//hip roll
   YoDouble kpFlexHip0, kpFlexHip1, kpFlexHip2, kpFlexHip3;//hip pitch
   YoDouble kpFlexKnee0, kpFlexKnee1, kpFlexKnee2, kpFlexKnee3;//knee pitch
   YoDouble kpFlexAnkle0, kpFlexAnkle1, kpFlexAnkle2, kpFlexAnkle3;//ankle pitch
   /*
    * leg controller derivative constants
    */
   double kdFlexHip, kdFlexKnee, kdFlexAnkle, kdAbduHip;

   private FloatingJoint q_rootJoint, qd_rootJoint, qdd_rootjoint;
   private YoDouble tau_abdHip0, tau_flexHip0, tau_abdHip1, tau_flexHip1, tau_abdHip2, tau_flexHip2, tau_abdHip3, tau_flexHip3;
   private YoDouble q_abdHip0, q_flexHip0, q_abdHip1, q_flexHip1, q_abdHip2, q_flexHip2, q_abdHip3, q_flexHip3;
   private YoDouble qd_abdHip0, qd_flexHip0, qd_abdHip1, qd_flexHip1, qd_abdHip2, qd_flexHip2, qd_abdHip3, qd_flexHip3;
   private YoDouble tau_flexKnee0, tau_flexKnee1, tau_flexKnee2, tau_flexKnee3;
   private YoDouble q_flexKnee0, q_flexKnee1, q_flexKnee2, q_flexKnee3, qd_flexKnee0, qd_flexKnee1, qd_flexKnee2, qd_flexKnee3;
   private YoDouble tau_flexAnkle0, tau_flexAnkle1, tau_flexAnkle2, tau_flexAnkle3;
   private YoDouble q_flexAnkle0, q_flexAnkle1, q_flexAnkle2, q_flexAnkle3, qd_flexAnkle0, qd_flexAnkle1, qd_flexAnkle2, qd_flexAnkle3;

   /*
   joint's angle for graph
   */
   private YoDouble theta00, theta01, theta02, theta03, theta10, theta11, theta12, theta13, theta20, theta21, theta22, theta23, theta30, theta31, theta32,
         theta33;
   int[] pawState;
   double pawXYZ[][] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
   double[][] legTheta = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
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
      initControl();

      ticksForDesiredForce.set(10);
      tickCounter.set(ticksForDesiredForce.getIntegerValue() + 1);

      thetaDebugFileSetting();
      initializeYoDoubleJointVariables(robot);

      waveGait = new GuaraWaveGait(pawState, setPointCounter);
      kinematics = new GuaraKinematics(robot, waveGait);
      assert waveGait != null;
      assert kinematics != null;
      pawState = new int[4];
      pawXYZ = waveGait.pawXYZ;

      // thetaDebugFileSetting();

      initializeYoDoubleJointThetas();
      initControl();
   }

   public void initControl()

   {
/*      double[] tempKpAbduHip = robot.getAbduHipKP();
      double[] tempKpFlexHip = robot.getFlexHipKP();
      double[] tempKpFlexKnee = robot.getFlexKneeKP();
      double[] tempKpFlexAnkle = robot.getFlexAnkleKP();
      kpAbduHip1.set(3.0);
      kpFlexHip1 = 4;
      kpFlexKnee1 = 3;
      kpFlexAnkle0 = tempKpFlexAnkle[0];
      kpFlexAnkle1 = tempKpFlexAnkle[1];
      kpFlexAnkle2 = tempKpFlexAnkle[2];
      kpFlexAnkle3 = tempKpFlexAnkle[3];
      kdAbduHip = 0.0;
      kdFlexHip = 0.0;//0.3;//3;
      kdFlexKnee = 0.0;//0.5;//1;//5;
      kdFlexAnkle = 0.0;//0.5;//2;//1;//5;
*/
      /*
       * tauAbduHipController.setProportionalGain(kpAbduHip);
       * tauFlexHipController.setProportionalGain(kpFlexHip);
       * tauFlexKneeController.setProportionalGain(kpFlexKnee);
       * tauFlexAnkleController.setProportionalGain(kpFlexAnkle);
       * tauAbduHipController.setDerivativeGain(kdAbduHip);
       * tauFlexHipController.setDerivativeGain(kdFlexHip);
       * tauFlexKneeController.setDerivativeGain(kdFlexKnee);
       * tauFlexAnkleController.setDerivativeGain(kdFlexAnkle);
       */
   }

   public void doControl()
   {
      ticks++;
      setPointCounter++; //SP counter increment
      setPointCounter = setPointCounter == waveGait.totalOfColumns ? 0 : setPointCounter;
      waveGait.getFootState(waveGait.getWaveGaitMatrix(), pawState, setPointCounter);


      /*
       * thetas for graphic output
       */

       /*
       * theta00.set(legTheta[0][0]); theta01.set(legTheta[0][1]);
       * theta02.set(legTheta[0][2]); theta03.set(legTheta[0][3]);
       * saveToDebugTheta(0);
       */
      /*
       * if (tickCounter.getIntegerValue() >
       * ticksForDesiredForce.getIntegerValue()) { waveGait.footPath(0,
       * setPointCounter, pawState[0]); waveGait.footPath(1, setPointCounter,
       * pawState[1]); waveGait.footPath(2, setPointCounter, pawState[2]);
       * waveGait.footPath(3, setPointCounter, pawState[3]);
       * kinematics.inverseKinematics(0, legTheta, waveGait); //leg 0
       * kinematics.inverseKinematics(1, legTheta, waveGait); //leg 1
       * kinematics.inverseKinematics(2, legTheta, waveGait); //leg 2
       * kinematics.inverseKinematics(3, legTheta, waveGait); //leg 3
       * theta01.set(legTheta[0][0]); theta02.set(legTheta[0][1]);
       * theta03.set(legTheta[0][2]); theta04.set(legTheta[0][3]);
       * saveToDebugTheta(0); tickCounter.set(0); } tickCounter.increment(); //
       * legsTunningForControl(); //these are fixed angles to test //
       * legsTunningLikeInGuaraWolf(); yoThetasForGraphing();
       * tau_abdHip0.set(tauAbduHipController.compute(q_abdHip0.getDoubleValue()
       * , legTheta[0][0], qd_abdHip0.getDoubleValue(), 0.0, dt));
       * tau_flexHip0.set(tauFlexHipController.compute(q_flexHip0.getDoubleValue
       * (), legTheta[0][1], qd_flexHip0.getDoubleValue(), 0.0, dt));
       * tau_flexKnee0.set(tauFlexKneeController.compute(q_flexKnee0.
       * getDoubleValue(), legTheta[0][2], qd_flexKnee0.getDoubleValue(), 0.0,
       * dt)); tau_flexAnkle0.set(tauFlexAnkleController.compute(q_flexAnkle0.
       * getDoubleValue(), legTheta[0][3], qd_flexAnkle0.getDoubleValue(), 0.0,
       * dt)); /* Running leg's position control
       */
      /*
       * BEGIN temporary joint angles and controller constants assignments to tes robot posture
       */
      double[] hipAbduKP = robot.getAbduHipKP();
      double[] hipFlexKP = robot.getAbduHipKP();
      double[] kneeFlexKP = robot.getAbduHipKP();
      double[] ankleFlexKP = robot.getAbduHipKP();

      kpAbduHip0.set(hipAbduKP[0]);
      kpAbduHip1.set(hipAbduKP[1]);
      kpAbduHip2.set(hipAbduKP[2]);
      kpAbduHip3.set(hipAbduKP[3]);

      kpFlexHip0.set(hipFlexKP[0]);
      kpFlexHip1.set(hipFlexKP[1]);
      kpFlexHip2.set(hipFlexKP[2]);
      kpFlexHip3.set(hipFlexKP[3]);

      kpFlexKnee0.set(kneeFlexKP[0]);
      kpFlexKnee1.set(kneeFlexKP[1]);
      kpFlexKnee2.set(kneeFlexKP[2]);
      kpFlexKnee3.set(kneeFlexKP[3]);

      kpFlexAnkle0.set(ankleFlexKP[0]);
      kpFlexAnkle1.set(ankleFlexKP[1]);
      kpFlexAnkle2.set(ankleFlexKP[2]);
      kpFlexAnkle3.set(ankleFlexKP[3]);

      double[] hipAbduAngle = robot.getAbduHipAngle();
      double[] hipFlexAngle = robot.getFlexHipAngle();
      double[] kneeFlexAngle = robot.getFlexKneeAngle();
      double[] ankleFlexAngle = robot.getFlexAnkleAngle();

      theta00.set(hipAbduAngle[0]);
      theta01.set(hipAbduAngle[1]);
      theta02.set(hipAbduAngle[2]);
      theta03.set(hipAbduAngle[3]);

      theta10.set(hipAbduAngle[0]);
      theta11.set(hipAbduAngle[1]);
      theta12.set(hipAbduAngle[2]);
      theta13.set(hipAbduAngle[3]);

      theta20.set(hipAbduAngle[0]);
      theta21.set(hipAbduAngle[1]);
      theta22.set(hipAbduAngle[2]);
      theta23.set(hipAbduAngle[3]);

      theta30.set(hipAbduAngle[0]);
      theta31.set(hipAbduAngle[1]);
      theta32.set(hipAbduAngle[2]);
      theta33.set(hipAbduAngle[2]);

      /*
       * END temporary joint angles and controller constants assignments to tes robot posture
       */
       tau_abdHip0.set(kpAbduHip0.getDoubleValue() * (hipAbduAngle[0]/* legTheta[0][0] */ - q_abdHip0.getDoubleValue()) + kdAbduHip * (0 - qd_abdHip0.getDoubleValue()));
      tau_abdHip1.set(kpAbduHip1.getDoubleValue() * (hipAbduAngle[1]/* legTheta[1][0] */ - q_abdHip1.getDoubleValue()) + kdAbduHip * (0 - qd_abdHip1.getDoubleValue()));
      tau_abdHip2.set(kpAbduHip2.getDoubleValue() * (hipAbduAngle[2]/* legTheta[2][0] */ - q_abdHip2.getDoubleValue()) + kdAbduHip * (0 - qd_abdHip2.getDoubleValue()));
      tau_abdHip3.set(kpAbduHip3.getDoubleValue() * (hipAbduAngle[3]/* legTheta[3][0] */ - q_abdHip3.getDoubleValue()) + kdAbduHip * (0 - qd_abdHip3.getDoubleValue()));

      tau_flexHip0.set(kpFlexHip0.getDoubleValue() * (hipFlexAngle[0]/* legTheta[0][1] */ - q_flexHip0.getDoubleValue()) - kdFlexHip * (0 - qd_flexHip0.getDoubleValue()));
      tau_flexHip1.set(kpFlexHip1.getDoubleValue() * (hipFlexAngle[1]/* legTheta[1][1] */ - q_flexHip1.getDoubleValue()) - kdFlexHip * (0 - qd_flexHip1.getDoubleValue()));
      tau_flexHip2.set(kpFlexHip2.getDoubleValue() * (hipFlexAngle[2]/* legTheta[2][1] */ - q_flexHip2.getDoubleValue()) - kdFlexHip * (0 - qd_flexHip2.getDoubleValue()));
      tau_flexHip3.set(kpFlexHip3.getDoubleValue() * (hipFlexAngle[3]/* legTheta[3][1] */ - q_flexHip3.getDoubleValue()) - kdFlexHip * (0 - qd_flexHip3.getDoubleValue()));

      tau_flexKnee0.set(kpFlexKnee0.getDoubleValue() * (kneeFlexAngle[0]/* legTheta[0][2] */ - q_flexKnee0.getDoubleValue())
            + kdFlexKnee * (0 - qd_flexKnee0.getDoubleValue()));
      tau_flexKnee1.set(kpFlexKnee1.getDoubleValue() * (kneeFlexAngle[1]/* legTheta[1][2] */ - q_flexKnee1.getDoubleValue())
            + kdFlexKnee * (0 - qd_flexKnee1.getDoubleValue()));
      tau_flexKnee2.set(kpFlexKnee2.getDoubleValue() * (kneeFlexAngle[2]/* legTheta[2][2] */ - q_flexKnee2.getDoubleValue())
            + kdFlexKnee * (0 - qd_flexKnee2.getDoubleValue()));
      tau_flexKnee3.set(kpFlexKnee3.getDoubleValue() * (kneeFlexAngle[3]/* legTheta[3][2] */ - q_flexKnee3.getDoubleValue())
            + kdFlexKnee * (0 - qd_flexKnee3.getDoubleValue()));

      tau_flexAnkle0.set(kpFlexAnkle0.getDoubleValue() * (ankleFlexAngle[0]/* legTheta[0][3] */ - q_flexAnkle0.getDoubleValue())
            + kdFlexAnkle * (0 - qd_flexAnkle0.getDoubleValue()));
      tau_flexAnkle1.set(kpFlexAnkle1.getDoubleValue() * (ankleFlexAngle[1]/* legTheta[1][3] */ - q_flexAnkle1.getDoubleValue())
            + kdFlexAnkle * (0 - qd_flexAnkle1.getDoubleValue()));
      tau_flexAnkle2.set(kpFlexAnkle2.getDoubleValue() * (ankleFlexAngle[2]/* legTheta[2][3] */ - q_flexAnkle2.getDoubleValue())
            + kdFlexAnkle * (0 - qd_flexAnkle2.getDoubleValue()));
      tau_flexAnkle3.set(kpFlexAnkle3.getDoubleValue() * (ankleFlexAngle[3]/* legTheta[3][3] */ - q_flexAnkle3.getDoubleValue())
            + kdFlexAnkle * (0 - qd_flexAnkle3.getDoubleValue()));

      /*
       * tau_abdHip0.set(tauAbdHipController.compute(q_abdHip0.getDoubleValue()
       * , legTheta[0][0], qd_abdHip0.getDoubleValue(), 0.0, dt));
       * tau_flexHip0.set(tauFlexHipController.compute(q_flexHip0.getDoubleValue
       * (), legTheta[0][1], qd_flexHip0.getDoubleValue(), 0.0, dt));
       * tau_flexKnee0.set(tauFlexKneeController.compute(q_flexKnee0.
       * getDoubleValue(), legTheta[0][2], qd_flexKnee0.getDoubleValue(), 0.0,
       * dt)); tau_flexAnkle0.set(tauFlexAnkleController.compute(q_flexAnkle0.
       * getDoubleValue(), legTheta[0][3], qd_flexAnkle0.getDoubleValue(), 0.0,
       * dt)); leg 1
       * tau_abdHip1.set(tauAbdHipController.compute(q_abdHip1.getDoubleValue()
       * , legTheta[1][0], qd_abdHip1.getDoubleValue(), 0.0, dt));
       * tau_flexHip1.set(tauFlexHipController.compute(q_flexHip1.getDoubleValue
       * (), legTheta[1][1], qd_flexHip1.getDoubleValue(), 0.0, dt));
       * tau_flexKnee1.set(tauFlexKneeController.compute(q_flexKnee1.
       * getDoubleValue(), legTheta[1][2], qd_flexKnee1.getDoubleValue(), 0.0,
       * dt)); tau_flexAnkle1.set(tauFlexAnkleController.compute(q_flexAnkle1.
       * getDoubleValue(), legTheta[1][3], qd_flexAnkle1.getDoubleValue(), 0.0,
       * dt)); leg 2
       * tau_abdHip2.set(tauAbdHipController.compute(q_abdHip2.getDoubleValue()
       * , legTheta[2][0], qd_abdHip2.getDoubleValue(), 0.0, dt));
       * tau_flexHip2.set(tauFlexHipController.compute(q_flexHip2.getDoubleValue
       * (), legTheta[2][1], qd_flexHip2.getDoubleValue(), 0.0, dt));
       * tau_flexKnee2.set(tauFlexKneeController.compute(q_flexKnee2.
       * getDoubleValue(), legTheta[2][1], qd_flexKnee2.getDoubleValue(), 0.0,
       * dt)); tau_flexAnkle2.set(tauFlexAnkleController.compute(q_flexAnkle2.
       * getDoubleValue(), legTheta[2][1], qd_flexAnkle2.getDoubleValue(), 0.0,
       * dt)); leg 3
       * tau_abdHip3.set(tauAbdHipController.compute(q_abdHip3.getDoubleValue()
       * , legTheta[3][0], qd_abdHip3.getDoubleValue(), 0.0, dt));
       * tau_flexHip3.set(tauFlexHipController.compute(q_flexHip3.getDoubleValue
       * (), legTheta[3][1], qd_flexHip3.getDoubleValue(), 0.0, dt));
       * tau_flexKnee3.set(tauFlexKneeController.compute(q_flexKnee3.
       * getDoubleValue(), legTheta[3][2], qd_flexKnee3.getDoubleValue(), 0.0,
       * dt)); tau_flexAnkle3.set(tauFlexAnkleController.compute(q_flexAnkle3.
       * getDoubleValue(), legTheta[3][3], qd_flexAnkle3.getDoubleValue(), 0.0,
       * dt));
       */
   }
   /**
    *
    */
   public void yoThetasForGraphing()
   {
      /*
       * initialize theta YoVariables for graphing
       */

      theta00.set(legTheta[0][0]);
      theta01.set(legTheta[0][1]);
      theta02.set(legTheta[0][2]);
      theta03.set(legTheta[0][3]);
      theta10.set(legTheta[1][0]);
      theta11.set(legTheta[1][1]);
      theta12.set(legTheta[1][2]);
      theta13.set(legTheta[1][3]);
      theta20.set(legTheta[2][0]);
      theta21.set(legTheta[2][1]);
      theta22.set(legTheta[2][2]);
      theta23.set(legTheta[2][3]);
      theta30.set(legTheta[3][0]);
      theta31.set(legTheta[3][1]);
      theta32.set(legTheta[3][2]);
      theta33.set(legTheta[3][3]);

/*      tau_abdHip3.set(tauAbduHipController.compute(q_abdHip3.getDoubleValue(), legTheta[3][0], qd_abdHip3.getDoubleValue(), 0.0, dt));
      tau_flexHip3.set(tauFlexHipController.compute(q_flexHip3.getDoubleValue(), legTheta[3][1], qd_flexHip3.getDoubleValue(), 0.0, dt));
      tau_flexKnee3.set(tauFlexKneeController.compute(q_flexKnee3.getDoubleValue(), legTheta[3][2], qd_flexKnee3.getDoubleValue(), 0.0, dt));
      tau_flexAnkle3.set(tauFlexAnkleController.compute(q_flexAnkle3.getDoubleValue(), legTheta[3][3], qd_flexAnkle3.getDoubleValue(), 0.0, dt));
*/
   }

   /**
    *
    */
   public void initializeYoDoubleJointThetas()
   {
      theta00 = new YoDouble("theta00", registry);
      theta01 = new YoDouble("theta01", registry);
      theta02 = new YoDouble("theta02", registry);
      theta03 = new YoDouble("theta03", registry);
      theta10 = new YoDouble("theta10", registry);
      theta11 = new YoDouble("theta11", registry);
      theta12 = new YoDouble("theta12", registry);
      theta13 = new YoDouble("theta13", registry);
      theta20 = new YoDouble("theta20", registry);
      theta21 = new YoDouble("theta21", registry);
      theta22 = new YoDouble("theta22", registry);
      theta23 = new YoDouble("theta23", registry);
      theta30 = new YoDouble("theta30", registry);
      theta31 = new YoDouble("theta31", registry);
      theta32 = new YoDouble("theta32", registry);
      theta33 = new YoDouble("theta33", registry);
   }

   /**
    * @param robot
    */
   public void initializeYoDoubleJointVariables(GuaraRobot robot)
   {

      q_rootJoint = (FloatingJoint) robot.getRootJoint();

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
       * begin: legTheta debug file initializations
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
      /* end: legTheta debug file initializations */
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
      bw.write("legTheta[0]" + "\t" + "q_flexHip" + "\t" + "legTheta[0][2]" + "\t" + "q_flexKnee0" + "\t" + "legTheta[0][3]" + "\t" + "q_flexAnkle0");
      if (ticks <= 1000)
      {
         bw.write(Double.toString(legTheta[0][1]) + "\t" + Double.toString(q_flexHip0.getValueAsDouble()) + "\t" + Double.toString(legTheta[0][2]) + "\t"
               + Double.toString(q_flexKnee0.getValueAsDouble()) + "\t" + Double.toString(legTheta[0][3]) + "\t"
               + Double.toString(q_flexAnkle0.getValueAsDouble()));
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

   public GuaraWaveGait getWaveGait()
   {
      return waveGait;
   }

}
