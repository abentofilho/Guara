package guara;

import java.util.ArrayList;

//import us.ihmc.robotics.Axis;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
//SimulationConstructionSet;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UniversalJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class GuaraRobot extends Robot
{
   private final ArrayList<GroundContactPoint> groundContactPoints = new ArrayList();
   private final Joint rootJoint, abdFlexHip0, flexKnee0, flexAnkle0, abdFlexHip1, flexKnee1, flexAnkle1, abdFlexHip2, flexKnee2, flexAnkle2, abdFlexHip3,
         flexKnee3, flexAnkle3;
   public static final double // robot's body height
   hBodyZ = 0.36;
   public static final double // body data TO VERIFY
   lBodyX = 0.6, lBodyY = 0.4, lBodyZ = 0.1, mBody = 10.0, IxxBody = 0.1, IyyBody = 0.1, IzzBody = 0.1;
   public static final double // thigh data
   lThighX = 0.04 / 2, lThighY = 0.06 / 3, lThighZ = 0.15, mThigh = 5.63, IxxThigh = inertiaMoment(lThighY, lThighZ),
         IyyThigh = inertiaMoment(lThighX, lThighZ), IzzThigh = inertiaMoment(lThighX, lThighY);
   public static final double // shank data
   lShankX = 0.04 / 2, lShankY = 0.06 / 3, lShankZ = 0.15, mShank = 3.6, IxxShank = inertiaMoment(lShankY, lShankZ), IyyShank = inertiaMoment(lShankX, lShankZ),
         IzzShank = inertiaMoment(lShankX, lShankY);
   public static final double // foot data TO VERIFY
   lFootX = 0.08, lFootY = 0.06 / 3, lFootZ = 0.1, mFoot = 1.0, IxxFoot = inertiaMoment(lFootY, lFootZ), IyyFoot = inertiaMoment(lFootX, lFootZ),
         IzzFoot = inertiaMoment(lFootX, lFootY);
   public static final double // gearmotor data
   hMotor = 0.12 / 2, rMotor = 0.034 / 2;
   public static final double // robot's data
   lRobot = 0.6, wRobot = 0.36,
         // joint's height
         hThigh = lThighZ + lShankZ + lFootZ, // hip
         hKnee = lShankZ + lFootZ, // knee
         hFoot = lFootZ; // ankle

   /*
    * joint angles like in guara's wolf legs
    */
   private double[] abdHipAngle = new double[4];
   private double[] flexHipAngle = new double[4];
   private double[] flexKneeAngle = new double[4];
   private double[] flexAnkleAngle = new double[4];
   /*
    * controller constants
    */
   private double[] abduHipKP = new double[4];
   private double[] flexHipKP = new double[4];
   private double[] flexKneeKP = new double[4];
   private double[] flexAnkleKP = new double[4];
   private double[] abduHipKD = new double[4];
   private double[] flexHipKD = new double[4];
   private double[] flexKneeKD = new double[4];
   private double[] flexAnkleKD = new double[4];

   public double robotHeight; //to be set for desired leg's config

   public GuaraRobot()
   {

      /*
       * legs are numbered: 0 front left; 1 hind left; 2 front right; 3 hind
       * right
       */
      super("Guara");
      rootJoint = new FloatingJoint("rootJoint", new Vector3D(0.0, 0.0, 0.0), this);
      Link bodyLink = body();
      rootJoint.setLink(bodyLink);
      this.addRootJoint(rootJoint);
      bodyLink.addCoordinateSystemToCOM(0.5);

      abdFlexHip0 = new UniversalJoint("abdHip0", "flexHip0", new Vector3D(lRobot / 2, wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdFlexHip0);
      Link tigh0 = thigh(0);
      abdFlexHip0.setLink(tigh0);
      //      tigh0.addCoordinateSystemToCOM(0.25);

      flexKnee0 = new PinJoint("flexKnee0", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdFlexHip0.addJoint(flexKnee0);
      Link shank0 = shank();
      flexKnee0.setLink(shank0);
      //      shank0.addCoordinateSystemToCOM(0.25);

      flexAnkle0 = new PinJoint("flexAnkle0", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee0.addJoint(flexAnkle0);
      Link foot0 = foot();
      flexAnkle0.setLink(foot0);
      //      foot0.addCoordinateSystemToCOM(0.25);

      // Hip Joint setup as Universal Joint from leg 1, and follow up joints,
      // knee and ankle
      abdFlexHip1 = new UniversalJoint("abdHip1", "flexHip1", new Vector3D(-lRobot / 2, wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdFlexHip1);
      Link tigh1 = thigh(1);
      abdFlexHip1.setLink(tigh1);
      //      tigh1.addCoordinateSystemToCOM(0.25);

      flexKnee1 = new PinJoint("flexKnee1", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdFlexHip1.addJoint(flexKnee1);
      Link shank1 = shank();
      flexKnee1.setLink(shank1);

      flexAnkle1 = new PinJoint("flexAnkle1", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee1.addJoint(flexAnkle1);
      Link foot1 = foot();
      flexAnkle1.setLink(foot1);
      //      foot1.addCoordinateSystemToCOM(0.25);

      // Hip Joint setup as Universal Joint from leg 2, and follow up joints,
      // knee and ankle
      abdFlexHip2 = new UniversalJoint("abdHip2", "flexHip2", new Vector3D(-lRobot / 2, -wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdFlexHip2);
      Link tigh2 = thigh(2);
      abdFlexHip2.setLink(tigh2);
      //      tigh2.addCoordinateSystemToCOM(0.25);

      flexKnee2 = new PinJoint("flexKnee2", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdFlexHip2.addJoint(flexKnee2);
      Link shank2 = shank();
      flexKnee2.setLink(shank2);

      flexAnkle2 = new PinJoint("flexAnkle2", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee2.addJoint(flexAnkle2);
      Link foot2 = foot();
      flexAnkle2.setLink(foot2);

      // Hip Joint setup as Universal Joint from leg 3, and follow up joints,
      // knee and ankle
      abdFlexHip3 = new UniversalJoint("abdHip3", "flexHip3", new Vector3D(lRobot / 2, -wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdFlexHip3);
      Link tigh3 = thigh(3);
      abdFlexHip3.setLink(tigh3);

      flexKnee3 = new PinJoint("flexKnee3", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdFlexHip3.addJoint(flexKnee3);
      Link shank3 = shank();
      flexKnee3.setLink(shank3);

      flexAnkle3 = new PinJoint("flexAnkle3", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee3.addJoint(flexAnkle3);
      Link foot3 = foot();
      flexAnkle3.setLink(foot3);

      // Add ground contact points

      GroundContactPoint gcToe0 = new GroundContactPoint("gcToe0", new Vector3D(0.0, 0.0, -lFootZ), this);
      flexAnkle0.addGroundContactPoint(gcToe0);
      groundContactPoints.add(gcToe0);
      GroundContactPoint gcHeel0 = new GroundContactPoint("gcHeel0", new Vector3D(0.0, 0.0, 0.0), this);
      flexAnkle0.addGroundContactPoint(gcHeel0);
      groundContactPoints.add(gcHeel0);

      GroundContactPoint gcToe1 = new GroundContactPoint("gcToe1", new Vector3D(0.0, 0.0, -lFootZ), this);
      groundContactPoints.add(gcToe1);
      flexAnkle1.addGroundContactPoint(gcToe1);
      GroundContactPoint gcHeel1 = new GroundContactPoint("gcHeel1", new Vector3D(0.0, 0.0, 0.0), this);
      groundContactPoints.add(gcHeel1);
      flexAnkle1.addGroundContactPoint(gcHeel1);

      GroundContactPoint gcToe2 = new GroundContactPoint("gcToe2", new Vector3D(0.0, 0.0, -lFootZ), this);
      flexAnkle2.addGroundContactPoint(gcToe2);
      groundContactPoints.add(gcToe2);
      GroundContactPoint gcHeel2 = new GroundContactPoint("gcHeel2", new Vector3D(0.0, 0.0, 0.0), this);
      flexAnkle2.addGroundContactPoint(gcHeel2);
      groundContactPoints.add(gcHeel2);

      GroundContactPoint gcToe3 = new GroundContactPoint("gcToe3", new Vector3D(0.0, 0.0, -lFootZ), this);
      flexAnkle3.addGroundContactPoint(gcToe3);
      groundContactPoints.add(gcToe3);
      GroundContactPoint gcHeel3 = new GroundContactPoint("gcHeel3", new Vector3D(0.0, 0.0, 0.0), this);
      flexAnkle3.addGroundContactPoint(gcHeel3);
      groundContactPoints.add(gcHeel3);

      LinearGroundContactModel ground = new LinearGroundContactModel(this, this.getRobotsYoVariableRegistry());
      ground.setZStiffness(2000.0);
      ground.setZDamping(1500.0);
      ground.setXYStiffness(50000.0);
      ground.setXYDamping(2000.0);
      ground.setGroundProfile3D(new FlatGroundProfile());
      this.setGroundContactModel(ground);

      legsLikeInGuaraWolf();

   }

   /**
    * next lines code should go to a gait class
    */
   public void legsLikeInGuaraWolf()
   {
      /*
       * robot legs like in guara wolf
       */
      abdHipAngle[0] = 0.0;
      abdHipAngle[1] = 0.0;
      abdHipAngle[2] = 0.0;
      abdHipAngle[3] = 0.0;

      flexHipAngle[0] = Math.PI / 12;
      flexHipAngle[1] = Math.PI / 12;
      flexHipAngle[2] = Math.PI / 12;
      flexHipAngle[3] = Math.PI / 12;

      flexKneeAngle[0] = Math.PI / 4;
      flexKneeAngle[1] = Math.PI / 4;
      flexKneeAngle[2] = Math.PI / 4;
      flexKneeAngle[3] = Math.PI / 4;
      /*
       * ankle joint angles
       */
      flexAnkleAngle[0] = (-flexHipAngle[0] + flexKneeAngle[0]) / 2;
      flexAnkleAngle[1] = -flexHipAngle[1] - flexKneeAngle[1];
      flexAnkleAngle[2] = -flexHipAngle[2] - flexKneeAngle[2];
      flexAnkleAngle[3] = (-flexHipAngle[2] + flexKneeAngle[2]) / 2;
      /*
       * hip joint angles
       */
      ((UniversalJoint) abdFlexHip0).setInitialState(abdHipAngle[0], 0, flexHipAngle[0], 0);
      ((UniversalJoint) abdFlexHip1).setInitialState(abdHipAngle[1], 0, -flexHipAngle[1], 0);
      ((UniversalJoint) abdFlexHip2).setInitialState(abdHipAngle[2], 0, -flexHipAngle[2], 0);
      ((UniversalJoint) abdFlexHip3).setInitialState(abdHipAngle[3], 0, flexHipAngle[3], 0);

      //Knee Joint Angles

      ((PinJoint) flexKnee0).setInitialState(-flexKneeAngle[0], 0);
      ((PinJoint) flexKnee1).setInitialState(flexKneeAngle[1], 0);
      ((PinJoint) flexKnee2).setInitialState(flexKneeAngle[2], 0);
      ((PinJoint) flexKnee3).setInitialState(-flexKneeAngle[3], 0);

      //Ankle Joint angles

      ((PinJoint) flexAnkle0).setInitialState((-flexHipAngle[0] + flexKneeAngle[0]) / 2, 0);
      ((PinJoint) flexAnkle1).setInitialState(-flexHipAngle[1] - flexKneeAngle[1], 0);
      ((PinJoint) flexAnkle2).setInitialState(-flexHipAngle[2] - flexKneeAngle[2], 0);
      ((PinJoint) flexAnkle3).setInitialState((-flexHipAngle[3] + flexKneeAngle[3]) / 2, 0);

      //body heigth

      robotHeight = lThighZ * Math.cos(flexHipAngle[0]) + lShankZ * Math.cos(flexKneeAngle[0] - flexHipAngle[0]) + lFootX * Math.cos(-flexHipAngle[0]);

      //set floating joint position
      ((FloatingJoint) rootJoint).setPosition(0.0, 0.0, robotHeight);
      /*
       * controller constants for this configuration
       */
      abduHipKP[0] = 3;//10;//300;
      abduHipKP[1] = 3;
      abduHipKP[2] = 3;
      abduHipKP[3] = 3;

      flexHipKP[0] = 4;//4;//2;//0.2;//250;//20;//
      flexHipKP[1] = 4;//8;//
      flexHipKP[2] = 4;//8;
      flexHipKP[3] = 4;

      flexKneeKP[0] = 30;//60;//2400;//240;//120;//
      flexKneeKP[1] = 60;//30;//
      flexKneeKP[2] = 60;//30;//
      flexKneeKP[3] = 30;//60;//2400;//240;//120;//

      flexAnkleKP[0] = 20;//8;//4;
      flexAnkleKP[1] = 40;//8;//4;
      flexAnkleKP[2] = 40;
      flexAnkleKP[3] = 20;

      abduHipKD[0] = 0.0;//0.3;//1;//3;
      abduHipKD[1] = 0.0;
      abduHipKD[2] = 0.0;
      abduHipKD[3] = 0.0;

      flexHipKD[0] = 0.4;//0.8;
      flexHipKD[1] = 0.0;
      flexHipKD[2] = 0.0;
      flexHipKD[3] = 0.4;//0.8;

      flexKneeKD[0]= 0.5;//6.0;//1;//5;
      flexKneeKD[1]= 0.0;
      flexKneeKD[2]= 0.0;
      flexKneeKD[3]= 0.5;//6.0;

      flexAnkleKD[0] = 0.0;//0.5;//2;//1;//5;
      flexAnkleKD[1] = 0.0;//0.5;//2;//1;//5;
      flexAnkleKD[2] = 0.0;//0.5;//2;//1;//5;
      flexAnkleKD[3] = 0.0;//0.5;//2;//1;//5;

   }

   /*
    * x axis is red, y axis is white, and z axis is blue.
    */
   private Link body()
   {
      Link ret = new Link("Body");
      ret.setMass(mBody);
      ret.setComOffset(0, 0, 0);
      ret.setMomentOfInertia(IxxBody, IyyBody, IzzBody);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(4 * lBodyX / 5, lBodyY, lBodyZ / 2, YoAppearance.RGBColorFrom8BitInts(180, 76, 0));
      // linkGraphics.addCoordinateSystem(1);
      ret.setLinkGraphics(linkGraphics);
      // roll DOF gearmotor legs 0 and 3 is backward
      linkGraphics.translate(lBodyX / 2 - hMotor, lBodyY / 2 - rMotor / 2, 0.0);
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      linkGraphics.identity();
      linkGraphics.translate(lBodyX / 2 - hMotor, -lBodyY / 2 + rMotor / 2, 0.0);
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      // roll DOF gearmotor legs 1 and 2 is forward
      linkGraphics.identity();
      linkGraphics.translate(-lBodyX / 2, lBodyY / 2 - rMotor / 2, 0.0);
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      linkGraphics.identity();
      linkGraphics.translate(-lBodyX / 2, -lBodyY / 2 + rMotor / 2, 0.0);
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      return ret;
   }

   private Link thigh(int legNumber)
   {
      Link ret = new Link("Thigh");
      ret.setMass(mThigh);
      ret.setComOffset(0, 0, lThighZ / 2);
      ret.setMomentOfInertia(IxxThigh, IyyThigh, IzzThigh);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0, 0, -lThighZ); // coxa
      linkGraphics.addCube(lThighX, lThighY, lThighZ, YoAppearance.DarkSlateGrey());//
      linkGraphics.translate(0, 0, lThighZ); // juntas do quadril
      linkGraphics.rotate(Math.PI / 2, Axis.X);
      linkGraphics.translate(0.0, 0.0, -hMotor / 2);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      linkGraphics.rotate(Math.PI / 2, Axis.Y);
      // linkGraphics.addCoordinateSystem(0.1);
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link shank()
   {
      Link ret = new Link("Shank");
      ret.setMass(mShank);
      ret.setComOffset(0.0, 0.0, lShankZ / 2);
      ret.setMomentOfInertia(IxxShank, IyyShank, IzzShank);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0, 0, -lShankZ); // shank
      linkGraphics.addCube(lShankX, lShankY, lShankZ, YoAppearance.DarkSlateGrey());
      linkGraphics.translate(0, 0, +lShankZ); // shank
      linkGraphics.rotate(Math.PI / 2, Axis.X);
      linkGraphics.translate(0.0, 0.0, -hMotor / 2);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link foot()
   {
      Link ret = new Link("Foot");
      ret.setMass(mFoot);
      ret.setComOffset(0.0, 0.0, lFootZ / 2);
      ret.setMomentOfInertia(0.0, IyyFoot, 0.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.identity();
      linkGraphics.translate(0, 0, -lFootZ); // feet
      linkGraphics.addCube(lFootX / 3, lFootY, lFootZ, YoAppearance.BlackMetalMaterial());
      linkGraphics.rotate(Math.PI / 2, Axis.X);
      linkGraphics.translate(0.0, lFootZ, 0.0);
      linkGraphics.translate(0.0, 0.0, -hMotor / 2);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   public UniversalJoint getAbdFlexHip0()
   {
      return (UniversalJoint) abdFlexHip0;
   }

   public PinJoint getFlexKnee0()
   {
      return (PinJoint) flexKnee0;
   }

   public PinJoint getFlexAnkle0()
   {
      return (PinJoint) flexAnkle0;
   }

   public UniversalJoint getAbdFlexHip1()
   {
      return (UniversalJoint) abdFlexHip1;
   }

   public PinJoint getFlexKnee1()
   {
      return (PinJoint) flexKnee1;
   }

   public PinJoint getFlexAnkle1()
   {
      return (PinJoint) flexAnkle1;
   }

   static double inertiaMoment(double sideOne, double sideTwo)
   {
      return (Math.pow(sideOne, 2) + Math.pow(sideTwo, 2));
   }

   public UniversalJoint getAbdFlexHip2()
   {
      return (UniversalJoint) abdFlexHip2;
   }

   public PinJoint getFlexKnee2()
   {
      return (PinJoint) flexKnee2;
   }

   public PinJoint getFlexAnkle2()
   {
      return (PinJoint) flexAnkle2;
   }

   public UniversalJoint getAbdFlexHip3()
   {
      return (UniversalJoint) abdFlexHip3;
   }

   public PinJoint getFlexKnee3()
   {
      return (PinJoint) flexKnee3;
   }

   public PinJoint getFlexAnkle3()
   {
      return (PinJoint) flexAnkle3;
   }

   public double a2()
   {
      return lThighZ;
   }

   public double a3()
   {
      return lShankZ;
   }

   double a4()
   {
      return lFootZ;
   }

   public GuaraRobot guaraRobot()
   {
      return this;
   }

   public Joint getRootJoint()
   {
      return rootJoint;
   }

   public double[] getAbduHipAngle()
   {
      return abdHipAngle;
   }

   public double[] getFlexHipAngle()
   {
      return flexHipAngle;
   }

   public double[] getFlexKneeAngle()
   {
      return flexKneeAngle;
   }

   public double[] getFlexAnkleAngle()
   {
      return flexAnkleAngle;
   }

   public double[] getAbduHipKP()
   {
      return abduHipKP;
   }

   public double[] getFlexHipKP()
   {
      return flexHipKP;
   }

   public double[] getFlexKneeKP()
   {
      return flexKneeKP;
   }

   public double[] getFlexAnkleKP()
   {
      return flexAnkleKP;
   }

   public double[] getAbduHipKD()
   {
      return abduHipKD;
   }

   public double[] getFlexHipKD()
   {
      return flexHipKD;
   }

   public double[] getFlexKneeKD()
   {
      return flexKneeKD;
   }

   public double[] getFlexAnkleKD()
   {
      return flexAnkleKD;
   }


}