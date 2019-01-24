package guara;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;


public class GuaraYoVariablesDefinition {

//	protected GuaraRobot rob;
	// floating joint - corpo

	private final YoVariableRegistry registry = new YoVariableRegistry("guaraYoVariables");
	public YoDouble q_bodyJoint_x = new YoDouble("q_BodyJoint_x", registry);
	public YoDouble q_bodyJoint_y = new YoDouble("q_BodyJoint_y", registry);
	public YoDouble q_bodyJoint_z = new YoDouble("q_BodyJoint_z", registry);
	public YoDouble qd_bodyJoint_x = new YoDouble("qd_BodyJoint_x", registry);
	public YoDouble qd_bodyJoint_y = new YoDouble("qd_BodyJoint_y", registry);
	public YoDouble qd_bodyJoint_z = new YoDouble("qd_BodyJoint_z", registry);
	public YoDouble qdd_bodyJoint_x = new YoDouble("qdd_BodyJoint_x", registry);
	public YoDouble qdd_bodyJoint_y = new YoDouble("qdd_BodyJoint_y", registry);
	public YoDouble qdd_bodyJoint_z = new YoDouble("qdd_BodyJoint_z", registry);
	public YoDouble q_bodyJoint_qs = new YoDouble("q_bodyJoint_qs", registry);
	public YoDouble q_bodyJoint_qx = new YoDouble("q_bodyJoint_qx", registry);
	public YoDouble q_bodyJoint_qy = new YoDouble("q_bodyJoint_qy", registry);
	public YoDouble q_bodyJoint_qz = new YoDouble("q_bodyJoint_qz", registry);
	public YoDouble qd_bodyJoint_wx = new YoDouble("qd_bodyJoint_wx", registry);
	public YoDouble qd_bodyJoint_wy = new YoDouble("qd_bodyJoint_wy", registry);
	public YoDouble qd_bodyJoint_wz = new YoDouble("qd_bodyJoint_wz", registry);
	public YoDouble qdd_bodyJoint_wx = new YoDouble("qdd_bodyJoint_wx", registry);
	public YoDouble qdd_bodyJoint_wy = new YoDouble("qdd_bodyJoint_wy", registry);
	public YoDouble qdd_bodyJoint_wz = new YoDouble("qdd_bodyJoint_wz", registry);


	// universal joints- quadris

	// quadril perna 0
	
	public YoDouble q_abdHip0X= new YoDouble("q_abdHip0X", registry);
	public YoDouble qd_abdHip0X= new YoDouble("qd_abdHip0X", registry);
	public YoDouble qdd_abdHip0X= new YoDouble("qdd_abdHip0X", registry);
	public YoDouble tau_abdHip0X= new YoDouble("tau_abdHip0X", registry);

	public YoDouble q_abdHip0Y= new YoDouble("q_abdHip0Y", registry);
	public YoDouble qd_abdHip0Y= new YoDouble("qd_abdHip0Y", registry);
	public YoDouble qdd_abdHip0Y= new YoDouble("qdd_abdHip0Y", registry);
	public YoDouble tau_abdHip0Y= new YoDouble("tau_abdHip0Y", registry);
	// quadril perna 1

	public YoDouble q_abdHip1X= new YoDouble("q_abdHip1X", registry);
	public YoDouble qd_abdHip1X= new YoDouble("qd_abdHip1X", registry);
	public YoDouble qdd_abdHip1X= new YoDouble("qdd_abdHip1X", registry);
	public YoDouble tau_abdHip1X= new YoDouble("tau_abdHip1X", registry);

	public YoDouble q_abdHip1Y= new YoDouble("q_abdHip1Y", registry);
	public YoDouble qd_abdHip1Y= new YoDouble("qd_abdHip1Y", registry);
	public YoDouble qdd_abdHip1Y= new YoDouble("qdd_abdHip1Y", registry);
	public YoDouble tau_abdHip1Y= new YoDouble("tau_abdHip1Y", registry);
	
	// quadril perna 2

	public YoDouble q_abdHip2X= new YoDouble("q_abdHip2X", registry);
	public YoDouble qd_abdHip2X= new YoDouble("qd_abdHip2X", registry);
	public YoDouble qdd_abdHip2X= new YoDouble("qdd_abdHip2X", registry);
	public YoDouble tau_abdHip2X= new YoDouble("tau_abdHip2X", registry);

	public YoDouble q_abdHip2Y= new YoDouble("q_abdHip2Y", registry);
	public YoDouble qd_abdHip2Y= new YoDouble("qd_abdHip2Y", registry);
	public YoDouble qdd_abdHip2Y= new YoDouble("qdd_abdHip2Y", registry);
	public YoDouble tau_abdHip2Y= new YoDouble("tau_abdHip2Y", registry);
	
	// quadril perna 3

	public YoDouble q_abdHip3X= new YoDouble("q_abdHip3X", registry);
	public YoDouble qd_abdHip3X= new YoDouble("qd_abdHip3X", registry);
	public YoDouble qdd_abdHip3X= new YoDouble("qdd_abdHip3X", registry);
	public YoDouble tau_abdHip3X= new YoDouble("tau_abdHip3X", registry);
	
	public YoDouble q_abdHip3Y= new YoDouble("q_abdHip3Y", registry);
	public YoDouble qd_abdHip3Y= new YoDouble("qd_abdHip3Y", registry);
	public YoDouble qdd_abdHip3Y= new YoDouble("qdd_abdHip3Y", registry);
	public YoDouble tau_abdHip3Y= new YoDouble("tau_abdHip3Y", registry);
	
	// pin joints - joelhos

	// joelho perna 0
	
	//public YoDouble q_flexKnee0, qd_flexKnee0, qdd_flexKnee0, tau_flexKnee0;
	public YoDouble q_flexKnee0 = new YoDouble("q_flexKnee0", registry); 
	public YoDouble	qd_flexKnee0 = new YoDouble("qd_flexKnee0", registry); 
	public YoDouble	qdd_flexKnee0 = new YoDouble("qdd_flexKnee0", registry); 
	public YoDouble	tau_flexKnee0 = new YoDouble("tau_flexKnee0", registry);

	// joelho perna 1

	//public YoDouble q_flexKnee1, qd_flexKnee1, qdd_flexKnee1, tau_flexKnee1;
	public YoDouble q_flexKnee1 = new YoDouble("q_flexKnee1", registry); 
	public YoDouble	qd_flexKnee1 = new YoDouble("qd_flexKnee1", registry); 
	public YoDouble	qdd_flexKnee1 = new YoDouble("qdd_flexKnee1", registry); 
	public YoDouble	tau_flexKnee1 = new YoDouble("tau_flexKnee1", registry);
	
	// joelho perna 2

	//public YoDouble q_flexKnee2, qd_flexKnee2, qdd_flexKnee2, tau_flexKnee2;
	public YoDouble q_flexKnee2 = new YoDouble("q_flexKnee2", registry);
	public YoDouble qd_flexKnee2 = new YoDouble("qd_flexKnee2", registry); 
	public YoDouble qdd_flexKnee2 = new YoDouble("qdd_flexKnee2", registry); 
	public YoDouble tau_flexKnee2 = new YoDouble("tau_flexKnee2", registry);
	
	// joelho perna 3

	//public YoDouble q_flexKnee3, qd_flexKnee3, qdd_flexKnee3, tau_flexKnee3;
	public YoDouble q_flexKnee3 = new YoDouble("q_flexKnee3", registry); 
	public YoDouble qd_flexKnee3 = new YoDouble("qd_flexKnee3", registry); 
	public YoDouble qdd_flexKnee3 = new YoDouble("qdd_flexKnee3", registry); 
	public YoDouble tau_flexKnee3 = new YoDouble("tau_flexKnee3", registry);
	
	// pin joints - tornozelos

	// tornozelo perna 0
	
	public YoDouble q_flexAnkle0, qd_flexAnkle0, qdd_flexAnkle0, tau_flexAnkle0;
	/*public YoDouble q_flexAnkle0 = new YoDouble("q_flexAnkle0", registry);
	public YoDouble	qd_flexAnkle0 = new YoDouble("q_flexAnkle0", registry); 
	public YoDouble qdd_flexAnkle0 = new YoDouble("q_flexAnkle0", registry); 
	public YoDouble tau_flexAnkle0 = new YoDouble("q_flexAnkle0", registry);*/

	// tornozelo perna 1

	public YoDouble q_flexAnkle1, qd_flexAnkle1, qdd_flexAnkle1, tau_flexAnkle1;
	/*public YoDouble q_flexAnkle1 = new YoDouble("q_flexAnkle1", registry);
	public YoDouble	qd_flexAnkle1 = new YoDouble("q_flexAnkle1", registry);
	public YoDouble qdd_flexAnkle1 = new YoDouble("q_flexAnkle1", registry); 
	public YoDouble	tau_flexAnkle1 = new YoDouble("q_flexAnkle1", registry);
	*/
	// tornozelo perna 2

	public YoDouble q_flexAnkle2, qd_flexAnkle2, qdd_flexAnkle2, tau_flexAnkle2;
	/*public YoDouble q_flexAnkle2 = new YoDouble("q_flexAnkle2", registry);
	public YoDouble qd_flexAnkle2 = new YoDouble("q_flexAnkle2", registry); 
	public YoDouble qdd_flexAnkle2 = new YoDouble("q_flexAnkle2", registry); 
	public YoDouble tau_flexAnkle2 = new YoDouble("q_flexAnkle2", registry);
	*/
	// tornozelo perna 3

	public YoDouble q_flexAnkle3, qd_flexAnkle3, qdd_flexAnkle3, tau_flexAnkle3;
	/*public YoDouble q_flexAnkle3 = new YoDouble("q_flexAnkle3", registry);
	public YoDouble	qd_flexAnkle3 = new YoDouble("q_flexAnkle3", registry);
	public YoDouble qdd_flexAnkle3 = new YoDouble("q_flexAnkle3", registry);
	public YoDouble tau_flexAnkle3 = new YoDouble("q_flexAnkle3", registry);
*/
	// construtor
	public GuaraYoVariablesDefinition() {
	}

	public GuaraYoVariablesDefinition(GuaraRobot rob) {

		// this.rob = rob;

		// variáveis da junta flutuante do quadril

		bodyJointYOVariables(rob);

		// variáveis da junta universal do quadril - perna 0

		q_abdHip0X = (YoDouble) rob.getVariable("q_abdHip0X");
		qd_abdHip0X = (YoDouble) rob.getVariable("qd_abdHip0X");
		qdd_abdHip0X = (YoDouble) rob.getVariable("qdd_abdHip0X");
		tau_abdHip0X = (YoDouble) rob.getVariable("tau_abdHip0X");
		q_abdHip0Y = (YoDouble) rob.getVariable("q_abdHip0Y");
		qd_abdHip0Y = (YoDouble) rob.getVariable("qd_abdHip0Y");
		qdd_abdHip0Y = (YoDouble) rob.getVariable("qdd_abdHip0Y");
		tau_abdHip0Y = (YoDouble) rob.getVariable("tau_abdHip0Y");

		// variáveis da junta universal do quadril - perna 1

		q_abdHip1X = (YoDouble) rob.getVariable("q_abdHip1X");
		qd_abdHip1X = (YoDouble) rob.getVariable("qd_abdHip1X");
		qdd_abdHip1X = (YoDouble) rob.getVariable("qdd_abdHip1X");
		tau_abdHip1X = (YoDouble) rob.getVariable("tau_abdHip1X");
		q_abdHip1Y = (YoDouble) rob.getVariable("q_abdHip1Y");
		qd_abdHip1Y = (YoDouble) rob.getVariable("qd_abdHip1Y");
		qdd_abdHip1Y = (YoDouble) rob.getVariable("qdd_abdHip1Y");
		tau_abdHip1Y = (YoDouble) rob.getVariable("tau_abdHip1Y");

		// variáveis da junta universal do quadril - perna 2

		q_abdHip2X = (YoDouble) rob.getVariable("q_abdHip2X");
		qd_abdHip2X = (YoDouble) rob.getVariable("qd_abdHip2X");
		qdd_abdHip2X = (YoDouble) rob.getVariable("qdd_abdHip2X");
		tau_abdHip2X = (YoDouble) rob.getVariable("tau_abdHip2X");
		q_abdHip2Y = (YoDouble) rob.getVariable("q_abdHip2Y");
		qd_abdHip2Y = (YoDouble) rob.getVariable("qd_abdHip2Y");
		qdd_abdHip2Y = (YoDouble) rob.getVariable("qdd_abdHip2Y");
		tau_abdHip2Y = (YoDouble) rob.getVariable("tau_abdHip2Y");

		// variáveis da junta universal do quadril - perna 3

		q_abdHip3X = (YoDouble) rob.getVariable("q_abdHip3X");
		qd_abdHip3X = (YoDouble) rob.getVariable("qd_abdHip3X");
		qdd_abdHip3X = (YoDouble) rob.getVariable("qdd_abdHip3X");
		tau_abdHip3X = (YoDouble) rob.getVariable("tau_abdHip3X");
		q_abdHip3Y = (YoDouble) rob.getVariable("q_abdHip3Y");
		qd_abdHip3Y = (YoDouble) rob.getVariable("qd_abdHip3Y");
		qdd_abdHip3Y = (YoDouble) rob.getVariable("qdd_abdHip3Y");
		tau_abdHip3Y = (YoDouble) rob.getVariable("tau_abdHip3Y");

		// variáveis da pin joint do joelho - perna 0

		q_flexKnee0 = (YoDouble) rob.getVariable("q_flexKnee0");
		qd_flexKnee0 = (YoDouble) rob.getVariable("qd_flexKnee0");
		qdd_flexKnee0 = (YoDouble) rob.getVariable("qdd_flexKnee0");
		tau_flexKnee0 = (YoDouble) rob.getVariable("tau_flexKnee0");

		// variáveis da pin joint do joelho - perna 1

		q_flexKnee1 = (YoDouble) rob.getVariable("q_flexKnee1");
		qd_flexKnee1 = (YoDouble) rob.getVariable("qd_flexKnee1");
		qdd_flexKnee1 = (YoDouble) rob.getVariable("qdd_flexKnee1");
		tau_flexKnee1 = (YoDouble) rob.getVariable("tau_flexKnee1");
		//
		// //variáveis da pin joint do joelho - perna 2
		//
		q_flexKnee2 = (YoDouble) rob.getVariable("q_flexKnee2");
		qd_flexKnee2 = (YoDouble) rob.getVariable("qd_flexKnee2");
		qdd_flexKnee2 = (YoDouble) rob.getVariable("qdd_flexKnee2");
		tau_flexKnee2 = (YoDouble) rob.getVariable("tau_flexKnee2");
		//
		// //variáveis da pin joint do joelho - perna 3
		//
		q_flexKnee3 = (YoDouble) rob.getVariable("q_flexKnee3");
		qd_flexKnee3 = (YoDouble) rob.getVariable("qd_flexKnee3");
		qdd_flexKnee3 = (YoDouble) rob.getVariable("qdd_flexKnee3");
		tau_flexKnee3 = (YoDouble) rob.getVariable("tau_flexKnee3");
		//
		// //variáveis da pin joint do tornozelo - perna 0
		//
		q_flexAnkle0 = (YoDouble) rob.getVariable("q_flexAnkle0");
		qd_flexAnkle0 = (YoDouble) rob.getVariable("qd_flexKnee0");
		qdd_flexAnkle0 = (YoDouble) rob.getVariable("qdd_flexKnee0");
		tau_flexAnkle0 = (YoDouble) rob.getVariable("tau_flexKnee0");
		//
		// //variáveis da pin joint do tornozelo - perna 1
		//
		q_flexAnkle1 = (YoDouble) rob.getVariable("q_flexAnkle1");
		qd_flexKnee1 = (YoDouble) rob.getVariable("qd_flexAnkle1");
		qdd_flexAnkle1 = (YoDouble) rob.getVariable("qdd_flexAnkle1");
		tau_flexAnkle1 = (YoDouble) rob.getVariable("tau_flexAnkle1");
		//
		// //variáveis da pin joint do tornozelo - perna 2
		//
		q_flexAnkle2 = (YoDouble) rob.getVariable("q_flexAnkle2");
		qd_flexAnkle2 = (YoDouble) rob.getVariable("qd_flexAnkle2");
		qdd_flexAnkle2 = (YoDouble) rob.getVariable("qdd_flexAnkle2");
		tau_flexAnkle2 = (YoDouble) rob.getVariable("tau_flexAnkle2");
		//
		// //variáveis da pin joint do tornozelo - perna 3
		//
		q_flexAnkle3 = (YoDouble) rob.getVariable("q_flexAnkle3");
		qd_flexAnkle3 = (YoDouble) rob.getVariable("qd_flexAnkle3");
		qdd_flexAnkle3 = (YoDouble) rob.getVariable("qdd_flexAnkle3");
		tau_flexAnkle3 = (YoDouble) rob.getVariable("tau_flexAnkle3");
	}

	private void bodyJointYOVariables(GuaraRobot rob) {
		q_bodyJoint_x = (YoDouble) rob.getVariable("q_bodyJoint_x");

		q_bodyJoint_x = (YoDouble) rob.getVariable("q_bodyJoint_x");
		q_bodyJoint_y = (YoDouble) rob.getVariable("q_bodyJoint_y");
		q_bodyJoint_z = (YoDouble) rob.getVariable("q_bodyJoint_z");
		qd_bodyJoint_x = (YoDouble) rob.getVariable("qd_bodyJoint_x");
		qd_bodyJoint_y = (YoDouble) rob.getVariable("qd_bodyJoint_y");
		qd_bodyJoint_z = (YoDouble) rob.getVariable("qd_bodyJoint_z");
		qdd_bodyJoint_x = (YoDouble) rob.getVariable("qdd_bodyJoint_x");
		qdd_bodyJoint_y = (YoDouble) rob.getVariable("qdd_bodyJoint_y");
		qdd_bodyJoint_z = (YoDouble) rob.getVariable("qdd_bodyJoint_z");
		q_bodyJoint_qs = (YoDouble) rob.getVariable("q_bodyJoint_qs");
		q_bodyJoint_qx = (YoDouble) rob.getVariable("q_bodyJoint_qx");
		q_bodyJoint_qy = (YoDouble) rob.getVariable("q_bodyJoint_qy");
		q_bodyJoint_qz = (YoDouble) rob.getVariable("q_bodyJoint_qz");
		qd_bodyJoint_wx = (YoDouble) rob.getVariable("qd_bodyJoint_wx");
		qd_bodyJoint_wy = (YoDouble) rob.getVariable("qd_bodyJoint_wy");
		qd_bodyJoint_wz = (YoDouble) rob.getVariable("qd_bodyJoint_wz");
		qdd_bodyJoint_wx = (YoDouble) rob.getVariable("qdd_bodyJoint_wx");
		qdd_bodyJoint_wy = (YoDouble) rob.getVariable("qdd_bodyJoint_wy");
		qdd_bodyJoint_wz = (YoDouble) rob.getVariable("qdd_bodyJoint_wz");
	}

}
