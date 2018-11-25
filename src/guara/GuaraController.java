package guara;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class GuaraController implements RobotController
{
   private GuaraRobot rob;

   public GuaraWaveGait a3;

   private final YoVariableRegistry registry = new YoVariableRegistry("guaraController");

   // constantes do controlador da perna

   /*
    * double Kp0, Kd0, Ki0, // junta 0 Kp1, Kd1, Ki1, // junta 1 Kp2, Kd2, Ki2,
    * // junta 2 Kp3, Kd3, Ki3; // junta 3
    */
   double k1, k2, k3, k4;
   double kd1, kd2, kd3, kd4;

   private final YoInteger tickCounter = new YoInteger("tickCounter", registry);
   private final YoInteger ticksForDesiredTorques = new YoInteger("ticksForDesiredTorques", registry);

   private YoDouble tau_abdHip0, tau_flexHip0, tau_abdHip1, tau_flexHip1, tau_abdHip2, tau_flexHip2, tau_abdHip3, tau_flexHip3;
   private YoDouble q_abdHip0, q_flexHip0, q_abdHip1, q_flexHip1, q_abdHip2, q_flexHip2, q_abdHip3, q_flexHip3;
   private YoDouble qd_abdHip0, qd_flexHip0, qd_abdHip1, qd_flexHip1, qd_abdHip2, qd_flexHip2, qd_abdHip3, qd_flexHip3;

   private YoDouble tau_flexKnee0, tau_flexKnee1, tau_flexKnee2, tau_flexKnee3;
   private YoDouble q_flexKnee0, q_flexKnee1, q_flexKnee2, q_flexKnee3, qd_flexKnee0, qd_flexKnee1, qd_flexKnee2, qd_flexKnee3;

   private YoDouble tau_flexAnkle0, tau_flexAnkle1, tau_flexAnkle2, tau_flexAnkle3;
   private YoDouble q_flexAnkle0, q_flexAnkle1, q_flexAnkle2, q_flexAnkle3, qd_flexAnkle0, qd_flexAnkle1, qd_flexAnkle2, qd_flexAnkle3;

   double[][] xyz = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};// coordenadas
   // 4
   // set points counter

   int i = 0;

   public double thetad;

   public GuaraController(GuaraRobot robot)
   { //, String name) {

      /* for cascading of torque controller */
      ticksForDesiredTorques.set(10);
      tickCounter.set(ticksForDesiredTorques.getIntegerValue() + 1);

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

      //	   super(rob);
      //		System.out.println("guaraController");
      //	      this.name = name;
      this.rob = robot;

      a3 = new GuaraWaveGait(128);
      assert a3 != null;
      // System.out.println("a3==null");
      // System.out.println(a3 == null);

      initControl();

   }

   public void initControl()
   {

      //legs' ground contact coordinates

      xyz[0][0] = 0.0;
      xyz[0][1] = 0.0;
      xyz[0][2] = -0.3; // robot height with straighten legs

      xyz[1][0] = 0.0;
      xyz[1][1] = 0.0;
      xyz[1][2] = -0.3; // robot height with straighten legs

      xyz[2][0] = 0.0;
      xyz[2][1] = 0.0;
      xyz[2][2] = -0.3; // robot height with straighten legs

      xyz[3][0] = 0.0;
      xyz[3][1] = 0.0;
      xyz[3][2] = -0.3; // robot height with straighten legs
   }

   public void doControl()
   {

      ticksForDesiredTorques.set(10);
      tickCounter.set(ticksForDesiredTorques.getIntegerValue() + 1);

      k1 = 250;
      k2 = 300;
      k3 = -150;
      k4 = 300;
      kd1 = 3;
      kd2 = 5;
      kd3 = 5;
      kd4 = 3;

      if (tickCounter.getIntegerValue() > ticksForDesiredTorques.getIntegerValue())
      {

         tau_abdHip0.set(k4 * (0 - q_abdHip0.getValueAsDouble()) + kd4 * (0 - qd_abdHip0.getValueAsDouble()));
         tau_abdHip1.set(k4 * (0 - q_abdHip1.getValueAsDouble()) + kd4 * (0 - qd_abdHip1.getValueAsDouble()));
         tau_abdHip2.set(k4 * (0 - q_abdHip2.getValueAsDouble()) + kd4 * (0 - qd_abdHip2.getValueAsDouble()));
         tau_abdHip3.set(k4 * (0 - q_abdHip3.getValueAsDouble()) + kd4 * (0 - qd_abdHip3.getValueAsDouble()));

         tau_flexHip0.set(k1 * (rob.phiY - q_flexHip0.getValueAsDouble()) + kd1 * (0 - qd_flexHip0.getValueAsDouble()));
         tau_flexHip1.set(k1 * (rob.phiY - q_flexHip1.getValueAsDouble()) + kd1 * (0 - qd_flexHip1.getValueAsDouble()));
         tau_flexHip2.set(k1 * (rob.phiY - q_flexHip2.getValueAsDouble()) + kd1 * (0 - qd_flexHip2.getValueAsDouble()));
         tau_flexHip3.set(k1 * (rob.phiY - q_flexHip3.getValueAsDouble()) + kd1 * (0 - qd_flexHip3.getValueAsDouble()));

         tau_flexKnee0.set(k2 * (rob.theta - q_flexKnee0.getValueAsDouble()) + kd2 * (0 - qd_flexKnee0.getValueAsDouble()));
         tau_flexKnee1.set(k2 * (rob.theta - q_flexKnee1.getValueAsDouble()) + kd2 * (0 - qd_flexKnee1.getValueAsDouble()));
         tau_flexKnee2.set(k2 * (rob.theta - q_flexKnee2.getValueAsDouble()) + kd2 * (0 - qd_flexKnee2.getValueAsDouble()));
         tau_flexKnee3.set(k2 * (rob.theta - q_flexKnee3.getValueAsDouble()) + kd2 * (0 - qd_flexKnee3.getValueAsDouble()));

         tau_flexAnkle0.set(k3 * (rob.psi + q_flexAnkle0.getValueAsDouble()) + kd3 * (0 - qd_flexAnkle0.getValueAsDouble()));
         tau_flexAnkle1.set(k3 * (rob.psi + q_flexAnkle1.getValueAsDouble()) + kd3 * (0 - qd_flexAnkle1.getValueAsDouble()));
         tau_flexAnkle2.set(k3 * (rob.psi + q_flexAnkle2.getValueAsDouble()) + kd3 * (0 - qd_flexAnkle2.getValueAsDouble()));
         tau_flexAnkle3.set(k3 * (rob.psi + q_flexAnkle3.getValueAsDouble()) + kd3 * (0 - qd_flexAnkle3.getValueAsDouble()));

         tickCounter.set(0);
      }
      tickCounter.increment();

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
