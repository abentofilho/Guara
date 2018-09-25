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
   lFootX = 0.08 / 3, lFootY = 0.06 / 3, lFootZ = 0.1, mFoot = 1.0, IxxFoot = inertiaMoment(lFootY, lFootZ), IyyFoot = inertiaMoment(lFootX, lFootZ),
         IzzFoot = inertiaMoment(lFootX, lFootY);
   public static final double // gearmotor data
   hMotor = 0.12 / 2, rMotor = 0.034 / 2;
   public static final double // robot's data
   lRobot = 0.6, wRobot = 0.36,
         // joint's height
         hThigh = lThighZ + lShankZ + lFootZ, // hip
         hKnee = lShankZ + lFootZ, // knee
         hFoot = lFootZ; // ankle
   Joint rootJoint;

   //Setting Variables for guara`s posture
   /*
    * Theta will be the Knee joint angle rotation Psi will be the Ankle joint
    * angle of rotation Phi will be the AbduHip joint Y axis angle of rotation
    * lThighZ equivale a l1 e lShankZ equivale a l2
    */

   public double theta, thetacount, psi, phiY, phiX, h;

   public GuaraRobot()
   {

      // legs are numbered: 0 front left; 1 hind left; 2 front right; 3 hind right
      super("Guara");

      h = Math.sqrt(2 * lThighZ * lShankZ * Math.cos(theta) + Math.pow(lThighZ, 2) + Math.pow(lShankZ, 2));

      //Setting Variables for guara`s posture;
      /*
       * phiX and phiY will be the abdFlexHip joint X and Y axis angle of rotation
       * theta will be the flexKnee joint rotation angle
       * psi will be the flexAnkle joint rotation angle
       * lThighZ equivale a l1 e lShankZ equivale a l2
       */

      //I have to previously define theta
      /*theta = Math.PI / 10;

      phiY = Math.asin(lShankZ * Math.sin(theta) / h); //PhiY will be the abdFlexHip joint Y axis angle of rotation
      phiX = Math.PI / 12; //PhiX will be the abdFlexHip joint X axis angle of rotation
      theta = -Math.PI / 4;   //Theta will be the flexKnee joint rotation angle
      psi = Math.acos((lThighZ) * Math.sin(theta) / h); // Psi will be the Ankle joint angle of rotation

      thetacount = -theta;*/

      rootJoint = new FloatingJoint("rootJoint", new Vector3D(0.0, 0.0, 0.0), this);

    //  ((FloatingJoint) rootJoint).setPosition(0.0, 0.0, /*hThigh*/h);      Link bodyLink = body();
      ((FloatingJoint) rootJoint).setPosition(0.0, 0.0, /* hThigh */h*2);
      Link bodyLink = body();
      rootJoint.setLink(bodyLink);
      this.addRootJoint(rootJoint);
      bodyLink.addCoordinateSystemToCOM(0.25);

      // Hip Joint setup as Universal Joint from leg 0, and follow up joints, knee and ankle
      UniversalJoint abdFlexHip0 = new UniversalJoint("abdHip0", "flexHip0", new Vector3D(lRobot / 2, wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdFlexHip0);
      Link tigh0 = thigh(0);
      abdFlexHip0.setLink(tigh0);
      tigh0.addCoordinateSystemToCOM(0.25);

      PinJoint flexKnee0 = new PinJoint("flexKnee0", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdFlexHip0.addJoint(flexKnee0);
      Link shank0 = shank();
      flexKnee0.setLink(shank0);
      shank0.addCoordinateSystemToCOM(0.25);
      PinJoint flexAnkle0 = new PinJoint("flexAnkle0", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee0.addJoint(flexAnkle0);
      Link foot0 = foot();
      flexAnkle0.setLink(foot0);

      
   // Hip Joint setup as Universal Joint from leg 1, and follow up joints, knee and ankle
      UniversalJoint abdHip1 = new UniversalJoint("abdHip1X", "abdHip1Y", new Vector3D(-lRobot / 2, wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdHip1);
      foot0.addCoordinateSystemToCOM(0.25);

      // Hip Joint setup as Universal Joint from leg 1, and follow up joints, knee and ankle
      //UniversalJoint abdFlexHip1 = new UniversalJoint("abdHip1", "flexHip1", new Vector3D(-lRobot / 2, wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      //rootJoint.addJoint(abdFlexHip1);

      Link tigh1 = thigh(1);
      abdFlexHip1.setLink(tigh1);

      PinJoint flexKnee1 = new PinJoint("flexKnee1", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdFlexHip1.addJoint(flexKnee1);
      Link shank1 = shank();
      flexKnee1.setLink(shank1);
      PinJoint flexAnkle1 = new PinJoint("flexAnkle1", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee1.addJoint(flexAnkle1);
      Link foot1 = foot();
      flexAnkle1.setLink(foot1);
//<<<<<<< master
      
   // Hip Joint setup as Universal Joint from leg 2, and follow up joints, knee and ankle
      UniversalJoint abdHip2 = new UniversalJoint("abdHip2X", "abdHip2Y", new Vector3D(-lRobot / 2, -wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdHip2);
=======

      // Hip Joint setup as Universal Joint from leg 2, and follow up joints, knee and ankle
      //UniversalJoint abdFlexHip2 = new UniversalJoint("abdHip2", "flexHip2", new Vector3D(-lRobot / 2, -wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      //rootJoint.addJoint(abdFlexHip2);
//>>>>>>> master
      Link tigh2 = thigh(2);
      abdFlexHip2.setLink(tigh2);

      PinJoint flexKnee2 = new PinJoint("flexKnee2", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdFlexHip2.addJoint(flexKnee2);
      Link shank2 = shank();
      flexKnee2.setLink(shank2);
//<<<<<<< master
      
//=======

//>>>>>>> master
      PinJoint flexAnkle2 = new PinJoint("flexAnkle2", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee2.addJoint(flexAnkle2);
      Link foot2 = foot();
      flexAnkle2.setLink(foot2);
//<<<<<<< master
      
   // Hip Joint setup as Universal Joint from leg 3, and follow up joints, knee and ankle
      UniversalJoint abdHip3 = new UniversalJoint("abdHip3X", "abdHip3Y", new Vector3D(lRobot / 2, -wRobot / 2, 0.0), this, Axis.X, Axis.Y);
      rootJoint.addJoint(abdHip3);
//=======

      // Hip Joint setup as Universal Joint from leg 3, and follow up joints, knee and ankle
 //     UniversalJoint abdFlexHip3 = new UniversalJoint("abdHip3", "flexHip3", new Vector3D(lRobot / 2, -wRobot / 2, 0.0), this, Axis.X, Axis.Y);
 //     rootJoint.addJoint(abdFlexHip3);
//>>>>>>> master
      Link tigh3 = thigh(3);
      abdFlexHip3.setLink(tigh3);

      PinJoint flexKnee3 = new PinJoint("flexKnee3", new Vector3D(0.0, 0.0, -lThighZ), this, Axis.Y);
      abdFlexHip3.addJoint(flexKnee3);
      Link shank3 = shank();
      flexKnee3.setLink(shank3);
//<<<<<<< master
      
//=======

//>>>>>>> master
      PinJoint flexAnkle3 = new PinJoint("flexAnkle3", new Vector3D(0.0, 0.0, -lShankZ), this, Axis.Y);
      flexKnee3.addJoint(flexAnkle3);
      Link foot3 = foot();
      flexAnkle3.setLink(foot3);
//<<<<<<< master
      
      //Add a contact points
//=======

      //Add ground contact points
//>>>>>>> master
      GroundContactPoint gc0 = new GroundContactPoint("gc0", new Vector3D(0.0, 0.0, -lFootZ), this);
      flexAnkle0.addGroundContactPoint(gc0);
      groundContactPoints.add(gc0);
      GroundContactPoint gc00 = new GroundContactPoint("gc00", new Vector3D(0.0, 0.0, 0.0), this);
      flexAnkle0.addGroundContactPoint(gc00);
      groundContactPoints.add(gc00);
      
      GroundContactPoint gc1 = new GroundContactPoint("gc1", new Vector3D(0.0, 0.0, -lFootZ), this);
      groundContactPoints.add(gc1);
      flexAnkle1.addGroundContactPoint(gc1);
//<<<<<<< master
      GroundContactPoint gc11 = new GroundContactPoint("gc11", new Vector3D(0.0, 0.0, 0.0), this);
      groundContactPoints.add(gc11);
      flexAnkle1.addGroundContactPoint(gc11);
      
//=======
//>>>>>>> master
      GroundContactPoint gc2 = new GroundContactPoint("gc2", new Vector3D(0.0, 0.0, -lFootZ), this);
      flexAnkle2.addGroundContactPoint(gc2);
      groundContactPoints.add(gc2);
      GroundContactPoint gc22 = new GroundContactPoint("gc22", new Vector3D(0.0, 0.0, 0.0), this);
      flexAnkle2.addGroundContactPoint(gc22);
      groundContactPoints.add(gc22);
      
      GroundContactPoint gc3 = new GroundContactPoint("gc3", new Vector3D(0.0, 0.0, -lFootZ), this);
      flexAnkle3.addGroundContactPoint(gc3);
      groundContactPoints.add(gc3);
      GroundContactPoint gc33 = new GroundContactPoint("gc33", new Vector3D(0.0, 0.0, 0.0), this);
      flexAnkle3.addGroundContactPoint(gc33);
      groundContactPoints.add(gc33);
      
      LinearGroundContactModel ground = new LinearGroundContactModel(this, this.getRobotsYoVariableRegistry());
      ground.setZStiffness(2000.0);
      ground.setZDamping(1500.0);
      ground.setXYStiffness(50000.0);
      ground.setXYDamping(2000.0);
      ground.setGroundProfile3D(new FlatGroundProfile());
      this.setGroundContactModel(ground);
//<<<<<<< master
      
      
      
    
     
      //Guara Ankle Joint angles
      
      flexAnkle0.setInitialState(-Math.PI/2, 0);
      flexAnkle1.setInitialState(-Math.PI/2, 0);
      flexAnkle2.setInitialState(-Math.PI/2, 0);
      flexAnkle3.setInitialState(-Math.PI/2, 0);
      
      //Guara Knee Joint Angles

      flexKnee0.setInitialState(theta, 0);
      flexKnee1.setInitialState(theta, 0);
      flexKnee2.setInitialState(theta, 0);
      flexKnee3.setInitialState(theta, 0);
      
      //Guara Ankle Joint angles
      h = Math.sqrt(Math.pow(lThighZ, 2) + Math.pow(lShankZ, 2) + 2*lThighZ*lShankZ*Math.cos(thetacount));
      psi = Math.acos(lThighZ*Math.sin(thetacount)/h) ;
      flexAnkle0.setInitialState(-psi, 0);
      flexAnkle1.setInitialState(-psi, 0);
      flexAnkle2.setInitialState(-psi, 0);
      flexAnkle3.setInitialState(-psi, 0);
      
      //Guara Hip Joint Y axis rotation angles
      phi = Math.asin(lShankZ*Math.sin(thetacount)/h);
      abdHip0.setInitialState(0, 0, phi, 0);
      abdHip1.setInitialState(0, 0, phi, 0);
      abdHip2.setInitialState(0, 0, phi, 0);
      abdHip3.setInitialState(0, 0, phi, 0);
      
      
//=======

      //Guara Ankle Joint angles; these should go to InitControl method

      //I have to previously define theta
//      theta = Math.PI / 10;
//
//      h = Math.sqrt(2 * lThighZ * lShankZ * Math.cos(theta) + Math.pow(lThighZ, 2) + Math.pow(lShankZ, 2));
//      phiY = Math.asin(lShankZ * Math.sin(theta) / h); //PhiY will be the abdFlexHip joint Y axis angle of rotation
//      phiX = Math.PI / 12; //PhiX will be the abdFlexHip joint X axis angle of rotation
//      theta = -Math.PI / 4;   //Theta will be the flexKnee joint rotation angle
//      psi = Math.acos((lThighZ) * Math.sin(theta) / h); // Psi will be the Ankle joint angle of rotation
//
//      thetacount = -theta;
//
//      flexAnkle0.setInitialState(-Math.PI / 2, 0);
//      flexAnkle1.setInitialState(-Math.PI / 2, 0);
//      flexAnkle2.setInitialState(-Math.PI / 2, 0);
//      flexAnkle3.setInitialState(-Math.PI / 2, 0);
//
//      //Guara Knee Joint Angles
//
//      flexKnee0.setInitialState(theta, 0);
//      flexKnee1.setInitialState(theta, 0);
//      flexKnee2.setInitialState(theta, 0);
//      flexKnee3.setInitialState(theta, 0);
//
//      //Guara Ankle Joint angles
//      h = Math.sqrt(Math.pow(lThighZ, 2) + Math.pow(lShankZ, 2) + 2 * lThighZ * lShankZ * Math.cos(thetacount));
//      psi = Math.acos(lThighZ * Math.sin(thetacount) / h);
//      flexAnkle0.setInitialState(-psi, 0);
//      flexAnkle1.setInitialState(-psi, 0);
//      flexAnkle2.setInitialState(-psi, 0);
//      flexAnkle3.setInitialState(-psi, 0);
//
//      //Guara Hip Joint Y axis rotation angles
//      phiY = Math.asin(lShankZ * Math.sin(thetacount) / h);
//      abdFlexHip0.setInitialState(phiX, 0, phiY, 0);
//      abdFlexHip1.setInitialState(phiX, 0, phiY, 0);
//      abdFlexHip2.setInitialState(phiX, 0, phiY, 0);
//      abdFlexHip3.setInitialState(phiX, 0, phiY, 0);
//>>>>>>> master
   }

   /**
    * @param abdFlexHip0
    * @param flexKnee0
    * @param flexAnkle0
    * @param abdFlexHip1
    * @param flexKnee1
    * @param flexAnkle1
    * @param abdFlexHip2
    * @param flexKnee2
    * @param flexAnkle2
    * @param abdFlexHip3
    * @param flexKnee3
    * @param flexAnkle3
    */
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
      linkGraphics.addCube(lFootX, lFootY, lFootZ, YoAppearance.BlackMetalMaterial());
      linkGraphics.rotate(Math.PI / 2, Axis.X);
      linkGraphics.translate(0.0, lFootZ, 0.0);
      linkGraphics.translate(0.0, 0.0, -hMotor / 2);
      linkGraphics.addCylinder(hMotor, rMotor, YoAppearance.RGBColorFrom8BitInts(70, 130, 180));
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   static double inertiaMoment(double sideOne, double sideTwo)
   {
      return (Math.pow(sideOne, 2) + Math.pow(sideTwo, 2));
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

   public double theta()
   {
      return theta;
   }

}