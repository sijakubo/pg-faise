package uni.oldenburg.server.agent;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;

@SuppressWarnings("serial")
public class RampOrderAgent extends Agent {
	public final static String NAME = "RampOrderAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	private int rampType = 0;
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
			rampType = ((ConveyorRamp) myConveyor).getRampType();
		}
		
		//addBehaviour(new AssignDestinationBehaviour());
		//addBehaviour(new AssignVehicleForPackageBehaviour());
		//addBehaviour(new CanTakePackageBehaviour());
		//addBehaviour(new ExistsDestinationBehaviour());
		//addBehaviour(new InitializePacketAgentBehaviour());
		//addBehaviour(new IsCapacityAvailableBehaviour());
		//addBehaviour(new NotifyPacketArrivalBehaviour());
		//addBehaviour(new ReserveSpaceBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(RampOrderAgent.NAME, conveyorID, szenarioID);
		AgentHelper.registerAgent(szenarioID, this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * Behaviour should receive the request of a Packageagent and should ask the Orderagents of a Storage if there is a Package for the given ID
	 * 
	 * @author Raschid
	 */
	private class CheckIfPackageExistsBehaviour extends CyclicBehaviour {

		@Override
		public void action() {
			
			
		}	
		
	}
	
	/**
	 * Behaviour should receive the request from an exit Orderagent and should ask the Packageagent if a package is stored, which 
	 * is needed by the exit and then answer the Orderagent of an exit if the Package exists or not
	 * @author Raschid
	 */
	private class CheckIfPackageIsStoredBehaviour extends CyclicBehaviour {

		@Override
		public void action() {
			
			
		}	
		
	}
	
	/**
	 * Behaviour should receive the answer from the Orderagent of Storage and tell its Packageagent if the package he requested exists 
	 * or not
	 * @author Raschid
	 */
	private class PackageAgentReceiveIfPackageIsStoredOrNot extends CyclicBehaviour {

		@Override
		public void action() {
			
			
		}	
		
	}
	
}
