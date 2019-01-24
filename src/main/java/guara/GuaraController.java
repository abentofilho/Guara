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
   YoDouble kdAbduHip0, kdFlexHip0, kdFlexKnee0, kdFlexAnkle0;
   YoDouble kdAbduHip1, kdFlexHip1, kdFlexKnee1, kdFlexAnkle1;
   YoDouble kdAbduHip2, kdFlexHip2, kdFlexKnee2, kdFlexAnkle2;
   YoDouble kdAbduHip3, kdFlexHip3, kdFlexKnee3, kdFlexAnkle3;

   private FloatingJoint q_rootJoint, qd_rootJoint, qdd_rootjoint;
   private YoDouble tau_abdHip0, tau_flexHip0, tau_abdHip1, tau_flexHip1, tau_abdHip2, tau_flexHip2, tau_abdHip3, tau_flexHip3;
   private YoDouble q_abdHip0, q_flexHip0, q_abdHip1, q_flexHip1, q_abdHip2, q_flexHip2, q_abdHip3, q_flexHip3;
   private YoDouble qd_abdHip0, qd_flexHip0, qd_abdHip1, qd_flexHip1, qd_abdHip2, qd_flexHip2, qd_abdHip3, qd_flexHip3;
   private YoDouble tau_flexKnee0, tau_flexKnee1, tau_flexKnee2, tau_flexKnee3;
   private YoDouble q_flexKnee0, q_flexKnee1, q_flexKnee2, q_flexKnee3, qd_flexKnee0, qd_flexKnee1, qd_flexKnee2, qd_flexKnee3;
   private YoDouble tau_flexAnkle0, tau_flexAnkle1, tau_flexAnkle2, tau_flexAnkle3;
   private YoDouble q_flexAnkle0, q_flexAnkle1, q_flexAnkle2, q_flexAnkle3, qd_flexAnkle0, qd_flexAnkle1, qd_flexAnkle2, qd_flexAnkle3;
   /*
    * joint's angle for graph
    */
   private YoDouble angleAbduHip0, angleFlexHip0, angleFlexKnee0, angleFlexAnkle0, angleAbduHip1, angleFlexHip1, angleFlexKnee1, angleFlexAnkle1, angleAbduHip2,
         angleFlexHip2, angleFlexKnee2, angleFlexAnkle2, angleAbduHip3, angleFlexHip3, angleFlexKnee3, angleFlexAnkle3;
   /*
    * pid errors as yodoubles for debug
    */
   private YoDouble errorAbduHip0, errorFlexHip0, errorFlexKnee0, errorFlexAnkle0, errorAbduHip1, errorFlexHip1, errorFlexKnee1, errorFlexAnkle1, errorAbduHip2,
         errorFlexHip2, errorFlexKnee2, errorFlexAnkle2, errorAbduHip3, errorFlexHip3, errorFlexKnee3, errorFlexAnkle3;
   /*
    * others doubles and ints
    */
   int[] pawState;
   double pawXYZ[][] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
   double[][] legTheta = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
   int setPointCounter = 0; //tick's counter for gait matrix adreeing
   int ticksForIncrementDeltaX = 128; //see velocity calculations
   int ticks = 0;
   double dt;
   /*
    * angles for debug
    */
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

      initializeGuaraWolfYoDoubleJointThetas();
      guaraWolfYoDoubleControllerConstants();
      initializeYoDoubleControllerErrors();

      initControl();
   }

   /**
    *
    */
   public void guaraWolfYoDoubleControllerConstants()
   {
      /*
       * initialize proportional controller constants
       */
      kpAbduHip0 = new YoDouble("kpAbduHip0", registry);
      kpAbduHip1 = new YoDouble("kpAbduHip1", registry);
      kpAbduHip2 = new YoDouble("kpAbduHip2", registry);
      kpAbduHip3 = new YoDouble("kpAbduHip3", registry);

      kpFlexHip0 = new YoDouble("kpFlexHip0", registry);
      kpFlexHip1 = new YoDouble("kpFlexHip1", registry);
      kpFlexHip2 = new YoDouble("kpFlexHip2", registry);
      kpFlexHip3 = new YoDouble("kpFlexHip3", registry);

      kpFlexKnee0 = new YoDouble("kpFlexKnee0", registry);
      kpFlexKnee1 = new YoDouble("kpFlexKnee1", registry);
      kpFlexKnee2 = new YoDouble("kpFlexKnee2", registry);
      kpFlexKnee3 = new YoDouble("kpFlexKnee3", registry);

      kpFlexAnkle0 = new YoDouble("kpFlexAnkle0", registry);
      kpFlexAnkle1 = new YoDouble("kpFlexAnkle1", registry);
      kpFlexAnkle2 = new YoDouble("kpFlexAnkle2", registry);
      kpFlexAnkle3 = new YoDouble("kpFlexAnkle3", registry);
      /*
       * initialize derivative controller constants
       */
      kdAbduHip0 = new YoDouble("kdAbduHip0", registry);
      kdAbduHip1 = new YoDouble("kdAbduHip1", registry);
      kdAbduHip2 = new YoDouble("kdAbduHip2", registry);
      kdAbduHip3 = new YoDouble("kdAbduHip3", registry);

      kdFlexHip0 = new YoDouble("kdFlexHip0", registry);
      kdFlexHip1 = new YoDouble("kdFlexHip1", registry);
      kdFlexHip2 = new YoDouble("kdFlexHip2", registry);
      kdFlexHip3 = new YoDouble("kdFlexHip3", registry);

      kdFlexKnee0 = new YoDouble("kdFlexKnee0", registry);
      kdFlexKnee1 = new YoDouble("kdFlexKnee1", registry);
      kdFlexKnee2 = new YoDouble("kdFlexKnee2", registry);
      kdFlexKnee3 = new YoDouble("kdFlexKnee3", registry);

      kdFlexAnkle0 = new YoDouble("kdFlexAnkle0", registry);
      kdFlexAnkle1 = new YoDouble("kdFlexAnkle1", registry);
      kdFlexAnkle2 = new YoDouble("kdFlexAnkle2", registry);
      kdFlexAnkle3 = new YoDouble("kdFlexAnkle3", registry);
      /*
       * sets local controllers constants
       */
      double[] abduHipKP = robot.getAbduHipKP();
      double[] flexHipKP = robot.getFlexHipKP();
      double[] flexKneeKP = robot.getFlexKneeKP();
      double[] flexAnkleKP = robot.getFlexAnkleKP();

      double[] abduHipKD = robot.getAbduHipKD();
      double[] flexHipKD = robot.getFlexHipKD();
      double[] flexKneeKD = robot.getFlexKneeKD();
      double[] flexAnkleKD = robot.getFlexAnkleKD();

      kpAbduHip0.set(abduHipKP[0]);
      kpAbduHip1.set(abduHipKP[1]);
      kpAbduHip2.set(abduHipKP[2]);
      kpAbduHip3.set(abduHipKP[3]);

      kpFlexHip0.set(flexHipKP[0]);
      kpFlexHip1.set(flexHipKP[1]);
      kpFlexHip2.set(flexHipKP[2]);
      kpFlexHip3.set(flexHipKP[3]);

      kpFlexKnee0.set(flexKneeKP[0]);
      kpFlexKnee1.set(flexKneeKP[1]);
      kpFlexKnee2.set(flexKneeKP[2]);
      kpFlexKnee3.set(flexKneeKP[3]);

      kpFlexAnkle0.set(flexAnkleKP[0]);
      kpFlexAnkle1.set(flexAnkleKP[1]);
      kpFlexAnkle2.set(flexAnkleKP[2]);
      kpFlexAnkle3.set(flexAnkleKP[3]);

      kdAbduHip0.set(abduHipKD[0]);
      kdAbduHip1.set(abduHipKD[1]);
      kdAbduHip2.set(abduHipKD[2]);
      kdAbduHip3.set(abduHipKD[3]);

      kdFlexHip0.set(flexHipKD[0]);
      kdFlexHip1.set(flexHipKD[1]);
      kdFlexHip2.set(flexHipKD[2]);
      kdFlexHip3.set(flexHipKD[3]);

      kdFlexKnee0.set(flexKneeKD[0]);
      kdFlexKnee1.set(flexKneeKD[1]);
      kdFlexKnee2.set(flexKneeKD[2]);
      kdFlexKnee3.set(flexKneeKD[3]);

      kdFlexAnkle0.set(flexAnkleKD[0]);
      kdFlexAnkle1.set(flexAnkleKD[1]);
      kdFlexAnkle2.set(flexAnkleKD[2]);
      kdFlexAnkle3.set(flexAnkleKD[3]);
   }

   public void initControl()

   {
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

   public double errorSign(double angleA, double angleB)
   {
      double errorSign = (Math.signum(angleA) >= 0 && Math.signum(angleB) >= 0) ? +1 : -1;
      return errorSign;
   }

   public void doControl()
   {
      ticks++;
      setPointCounter++; //SP counter increment
      setPointCounter = setPointCounter == waveGait.totalOfColumns ? 0 : setPointCounter;
      waveGait.getFootState(waveGait.getWaveGaitMatrix(), pawState, setPointCounter);
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
       * angleFlexHip0.set(legTheta[0][0]); angleFlexKnee3.set(legTheta[0][1]);
       * angleFlexAnkle0.set(legTheta[0][2]); theta04.set(legTheta[0][3]);
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
       * TEMPORARY define pid errors as YoDoubles for debug
       */
      double sign;
      errorAbduHip0.set(angleAbduHip0.getDoubleValue() - q_abdHip0.getDoubleValue());
      errorAbduHip1.set(angleAbduHip1.getDoubleValue() - q_abdHip1.getDoubleValue());
      errorAbduHip2.set(angleAbduHip2.getDoubleValue() - q_abdHip2.getDoubleValue());
      errorAbduHip3.set(angleAbduHip3.getDoubleValue() - q_abdHip3.getDoubleValue());

      sign = errorSign(angleFlexHip0.getDoubleValue(), q_flexHip0.getDoubleValue());
      errorFlexHip0.set(sign * angleFlexHip0.getDoubleValue() - q_flexHip0.getDoubleValue());
      errorFlexHip1.set(angleFlexHip1.getDoubleValue() - q_flexHip1.getDoubleValue());
      errorFlexHip2.set(angleFlexHip2.getDoubleValue() - q_flexHip2.getDoubleValue());
      sign = errorSign(angleFlexHip3.getDoubleValue(), q_flexHip3.getDoubleValue());
      errorFlexHip3.set(sign * angleFlexHip3.getDoubleValue() - q_flexHip3.getDoubleValue());

      sign = errorSign(angleFlexKnee0.getDoubleValue(), q_flexKnee0.getDoubleValue());
      errorFlexKnee0.set(sign * angleFlexKnee0.getDoubleValue() - q_flexKnee0.getDoubleValue());
      errorFlexKnee1.set(angleFlexKnee1.getDoubleValue() - q_flexKnee1.getDoubleValue());
      errorFlexKnee2.set(angleFlexKnee2.getDoubleValue() - q_flexKnee2.getDoubleValue());
      sign = errorSign(angleFlexKnee3.getDoubleValue(), q_flexKnee3.getDoubleValue());
      errorFlexKnee3.set(sign * (angleFlexKnee3.getDoubleValue() - q_flexKnee3.getDoubleValue()));

      sign = errorSign(angleFlexAnkle0.getDoubleValue(), q_flexAnkle0.getDoubleValue());
      errorFlexAnkle0.set(angleFlexAnkle0.getDoubleValue() - q_flexAnkle0.getDoubleValue());
      errorFlexAnkle1.set(angleFlexAnkle1.getDoubleValue() - q_flexAnkle1.getDoubleValue());
      errorFlexAnkle2.set(angleFlexAnkle2.getDoubleValue() - q_flexAnkle2.getDoubleValue());
      sign = errorSign(angleFlexAnkle3.getDoubleValue(), q_flexAnkle3.getDoubleValue());
      errorFlexAnkle3.set(angleFlexAnkle3.getDoubleValue() - q_flexAnkle3.getDoubleValue());
      errorFlexAnkle3.set(angleFlexAnkle3.getDoubleValue() - q_flexAnkle3.getDoubleValue());
      /*
       * controllers
       */
      tau_abdHip0.set(kpAbduHip0.getDoubleValue() * errorAbduHip0.getDoubleValue() + kdAbduHip0.getDoubleValue() * (0 - qd_abdHip0.getDoubleValue()));
      tau_abdHip1.set(kpAbduHip1.getDoubleValue() * errorAbduHip1.getDoubleValue() + kdAbduHip1.getDoubleValue() * (0 - qd_abdHip1.getDoubleValue()));
      tau_abdHip2.set(kpAbduHip2.getDoubleValue() * errorAbduHip2.getDoubleValue() + kdAbduHip2.getDoubleValue() * (0 - qd_abdHip2.getDoubleValue()));
      tau_abdHip3.set(kpAbduHip3.getDoubleValue() * errorAbduHip3.getDoubleValue() + kdAbduHip3.getDoubleValue() * (0 - qd_abdHip3.getDoubleValue()));

      tau_flexHip0.set(kpFlexHip0.getDoubleValue() * errorFlexHip0.getDoubleValue() - kdFlexHip0.getDoubleValue() * (0 - qd_flexHip0.getDoubleValue()));
      tau_flexHip1.set(kpFlexHip1.getDoubleValue() * errorFlexHip1.getDoubleValue() - kdFlexHip1.getDoubleValue() * (0 - qd_flexHip1.getDoubleValue()));
      tau_flexHip2.set(kpFlexHip2.getDoubleValue() * errorFlexHip2.getDoubleValue() - kdFlexHip2.getDoubleValue() * (0 - qd_flexHip2.getDoubleValue()));
      tau_flexHip3.set(kpFlexHip3.getDoubleValue() * errorFlexHip3.getDoubleValue() - kdFlexHip3.getDoubleValue() * (0 - qd_flexHip3.getDoubleValue()));

      tau_flexKnee0.set(kpFlexKnee0.getDoubleValue() * errorFlexKnee0.getDoubleValue() + kdFlexKnee0.getDoubleValue() * (0 - qd_flexKnee0.getDoubleValue()));
      tau_flexKnee1.set(kpFlexKnee1.getDoubleValue() * errorFlexKnee1.getDoubleValue() + kdFlexKnee1.getDoubleValue() * (0 - qd_flexKnee1.getDoubleValue()));
      tau_flexKnee2.set(kpFlexKnee2.getDoubleValue() * errorFlexKnee2.getDoubleValue() + kdFlexKnee2.getDoubleValue() * (0 - qd_flexKnee2.getDoubleValue()));
      tau_flexKnee3.set(kpFlexKnee3.getDoubleValue() * errorFlexKnee0.getDoubleValue() + kdFlexKnee3.getDoubleValue() * (0 - qd_flexKnee3.getDoubleValue()));

      tau_flexAnkle0.set(kpFlexAnkle0.getDoubleValue() * errorFlexAnkle0.getDoubleValue()
            + kdFlexAnkle0.getDoubleValue() * (0 - qd_flexAnkle0.getDoubleValue()));
      tau_flexAnkle1.set(kpFlexAnkle1.getDoubleValue() * errorFlexAnkle1.getDoubleValue()
            + kdFlexAnkle1.getDoubleValue() * (0 - qd_flexAnkle1.getDoubleValue()));
      tau_flexAnkle2.set(kpFlexAnkle2.getDoubleValue() * errorFlexAnkle2.getDoubleValue()
            + kdFlexAnkle2.getDoubleValue() * (0 - qd_flexAnkle2.getDoubleValue()));
      tau_flexAnkle3.set(kpFlexAnkle3.getDoubleValue() * errorFlexAnkle3.getDoubleValue()
            + kdFlexAnkle3.getDoubleValue() * (0 - qd_flexAnkle3.getDoubleValue()));

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

   /*
    *
    */
   public void initializeYoDoubleControllerErrors()
   {
      errorAbduHip0 = (YoDouble) new YoDouble("errorAbduHip0", registry);
      errorFlexHip0 = (YoDouble) new YoDouble("errorFlexHip0", registry);
      errorFlexKnee0 = (YoDouble) new YoDouble("errorFlexKnee0", registry);
      errorFlexAnkle0 = (YoDouble) new YoDouble("errorFlexAnkle0", registry);

      errorAbduHip1 = (YoDouble) new YoDouble("errorAbduHip1", registry);
      errorFlexHip1 = (YoDouble) new YoDouble("errorFlexHip1", registry);
      errorFlexKnee1 = (YoDouble) new YoDouble("errorFlexKnee1", registry);
      errorFlexAnkle1 = (YoDouble) new YoDouble("errorFlexAnkle1", registry);

      errorAbduHip2 = (YoDouble) new YoDouble("errorAbduHip2", registry);
      errorFlexHip2 = (YoDouble) new YoDouble("errorFlexHip2", registry);
      errorFlexKnee2 = (YoDouble) new YoDouble("errorFlexKnee2", registry);
      errorFlexAnkle2 = (YoDouble) new YoDouble("errorFlexAnkle2", registry);

      errorAbduHip3 = (YoDouble) new YoDouble("errorAbduHip3", registry);
      errorFlexHip3 = (YoDouble) new YoDouble("errorFlexHip3", registry);
      errorFlexKnee3 = (YoDouble) new YoDouble("errorFlexKnee3", registry);
      errorFlexAnkle3 = (YoDouble) new YoDouble("errorFlexAnkle3", registry);
   }

   /**
    *
    */
   public void initializeGuaraWolfYoDoubleJointThetas()
   {
      /*
       * leg's joint angles YoDoubles definition
       */
      angleAbduHip0 = new YoDouble("angleAbduHip0", registry);
      angleFlexHip0 = new YoDouble("angleFlexHip0", registry);
      angleFlexKnee0 = new YoDouble("angleFlexKnee0", registry);
      angleFlexAnkle0 = new YoDouble("angleFlexAnkle0", registry);

      angleAbduHip1 = new YoDouble("angleAbduHip1", registry);
      angleFlexHip1 = new YoDouble("angleFlexHip1", registry);
      angleFlexKnee1 = new YoDouble("angleFlexKnee1", registry);
      angleFlexAnkle1 = new YoDouble("angleFlexAnkle1", registry);

      angleAbduHip2 = new YoDouble("angleAbduHip2", registry);
      angleFlexHip2 = new YoDouble("angleFlexHip2", registry);
      angleFlexKnee2 = new YoDouble("angleFlexKnee2", registry);
      angleFlexAnkle2 = new YoDouble("angleFlexAnkle2", registry);

      angleAbduHip3 = new YoDouble("angleAbduHip3", registry);
      angleFlexHip3 = new YoDouble("angleFlexHip3", registry);
      angleFlexKnee3 = new YoDouble("angleFlexKnee3", registry);
      angleFlexAnkle3 = new YoDouble("angleFlexAnkle3", registry);
      /*
       * get joint angles assignments to this robot posture
       */
      double[] hipAbduAngle = robot.getAbduHipAngle();
      double[] hipFlexAngle = robot.getFlexHipAngle();
      double[] kneeFlexAngle = robot.getFlexKneeAngle();
      double[] ankleFlexAngle = robot.getFlexAnkleAngle();
      /*
       * initialize joit's YoDoubles
       */
      angleAbduHip0.set(hipAbduAngle[0]); //leg 0
      angleAbduHip1.set(hipAbduAngle[1]); //leg 1
      angleAbduHip2.set(hipAbduAngle[2]); //leg 2
      angleAbduHip3.set(hipAbduAngle[3]); //leg 3

      angleFlexHip0.set(hipFlexAngle[0]);
      angleFlexHip1.set(hipFlexAngle[1]);
      angleFlexHip2.set(hipFlexAngle[2]);
      angleFlexHip3.set(hipFlexAngle[3]);

      angleFlexKnee0.set(kneeFlexAngle[0]);
      angleFlexKnee1.set(kneeFlexAngle[1]);
      angleFlexKnee2.set(kneeFlexAngle[2]);
      angleFlexKnee3.set(kneeFlexAngle[3]);

      angleFlexAnkle0.set(ankleFlexAngle[0]);
      angleFlexAnkle1.set(ankleFlexAngle[1]);
      angleFlexAnkle2.set(ankleFlexAngle[2]);
      angleFlexAnkle3.set(ankleFlexAngle[3]);
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
