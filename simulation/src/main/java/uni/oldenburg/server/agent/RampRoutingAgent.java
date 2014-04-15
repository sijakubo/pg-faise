package uni.oldenburg.server.agent;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;

@SuppressWarnings("serial")
public class RampRoutingAgent extends Agent {
	public final static String NAME = "RampRoutingAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	
	private Logger logger = Logger.getLogger(RampRoutingAgent.class);
	
	/**
     * @author Matthias
     */	
	// init
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenarioID = (Integer) args[0];
			
			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
		}
		
		addBehaviour(new AssignVehicleForPackageBehaviour());
		addBehaviour(new FindVehicleBehaviour());
		addBehaviour(new ReceiveEstimationBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(RampRoutingAgent.NAME, conveyorID, szenarioID);		
		AgentHelper.registerAgent(szenarioID, this, nickname);
		
		logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}

	private class AssignVehicleForPackageBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	private class FindVehicleBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
	
	private class ReceiveEstimationBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
}
