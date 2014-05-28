package uni.oldenburg.server.agent;

import jade.core.Agent;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.Szenario;

@SuppressWarnings("serial")
public class RampOrderAgent extends Agent {
	public final static String NAME = "RampOrderAgent";

	private int conveyorID = 0;
	private Szenario szenario;
	private Logger logger = Logger.getLogger(RampOrderAgent.class);

	/**
	 * @author Matthias
	 */
	protected void setup() {
		Object[] args = getArguments();
		if (args != null) {
			szenario = (Szenario) args[0];

			Conveyor myConveyor = (Conveyor) args[1];
			conveyorID = myConveyor.getID();
		}
		
		String nickname = AgentHelper.getUniqueNickname(RampOrderAgent.NAME, conveyorID, szenario.getId());
		AgentHelper.registerAgent(szenario.getId(), this, nickname);

		if (Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}

	// destructor
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
}
