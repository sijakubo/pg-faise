package uni.oldenburg.server.agent;

import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;

@SuppressWarnings("serial")
public class PacketAgent extends Agent {
	public final static String NAME = "PacketAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	
	//private Logger logger = Logger.getLogger(PacketAgent.class);
	
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
		
		//addBehaviour(new AssignDestinationBehaviour());
		//addBehaviour(new ExistsDestinationBehaviour());
		//addBehaviour(new FindVehicleBehaviour());
		
		String nickname = AgentHelper.getUniqueNickname(PacketAgent.NAME, conveyorID, szenarioID);
		AgentHelper.registerAgent(szenarioID, this, nickname);
		
		//logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
}
