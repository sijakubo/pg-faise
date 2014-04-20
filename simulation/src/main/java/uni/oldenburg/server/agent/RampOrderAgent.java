package uni.oldenburg.server.agent;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;

@SuppressWarnings("serial")
public class RampOrderAgent extends Agent {
	public final static String NAME = "RampOrderAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	
	private Logger logger = Logger.getLogger(RampOrderAgent.class);
	
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
		
		addBehaviour(new AssignDestinationBehaviour());
		addBehaviour(new AssignVehicleForPackageBehaviour());
		addBehaviour(new CanTakePackageBehaviour());
		addBehaviour(new ExistsDestinationBehaviour());
		addBehaviour(new InitializePacketAgentBehaviour());
		addBehaviour(new IsCapacityAvailableBehaviour());
		addBehaviour(new NotifyPacketArrivalBehaviour());
		addBehaviour(new ReserveSpaceBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(RampOrderAgent.NAME, conveyorID, szenarioID);
		AgentHelper.registerAgent(szenarioID, this, nickname);
		
		if(Debugging.showAgentStartupMessages)logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}

	private class AssignDestinationBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	private class AssignVehicleForPackageBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	private class CanTakePackageBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	private class ExistsDestinationBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	private class InitializePacketAgentBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	private class IsCapacityAvailableBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}

	private class NotifyPacketArrivalBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
	
	private class ReserveSpaceBehaviour extends Behaviour {

		public void action() {
			
		}

		public boolean done() {
			return false;
		}
	}
}
