package uni.oldenburg.server.agent;

import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

/**
 * @author Matthias
 */
@SuppressWarnings("serial")
public class PackageAgent extends Agent {
	public final static String NAME = "PackageAgent";
	
	private int conveyorID = 0;
	private int szenarioID = 0;
	
	private List<PackageData> lstPackage = new ArrayList<PackageData>();
	
	private Logger logger = Logger.getLogger(PackageAgent.class);
	
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
		
		addBehaviour(new AddPackageBehaviour(MessageTemplate.MatchPerformative(MessageType.ADD_PACKAGE)));
		addBehaviour(new GetPackageCountBehaviour(MessageTemplate.MatchPerformative(MessageType.GET_PACKAGE_COUNT)));
		addBehaviour(new RemovePackageBehaviour(MessageTemplate.MatchPerformative(MessageType.REMOVE_PACKAGE)));
		
		String nickname = AgentHelper.getUniqueNickname(PackageAgent.NAME, conveyorID, szenarioID);
		AgentHelper.registerAgent(szenarioID, this, nickname);
		
		if(Debugging.showAgentStartupMessages)
			logger.log(Level.INFO, nickname + " started");
	}
	
	// destructor 
	protected void takeDown() {
		AgentHelper.unregister(this);
	}
	
	/**
	 * add package to this agent
	 * 
     * @author Matthias
     */
	private class AddPackageBehaviour extends CyclicReceiverBehaviour {		
		protected AddPackageBehaviour(MessageTemplate mt) {
			super(mt);
		}		

		public void onMessage(ACLMessage msg) throws UnreadableException {
			PackageAgent currentAgent = (PackageAgent)myAgent;
			PackageData myPackage = (PackageData)msg.getContentObject();
			
			currentAgent.lstPackage.add(myPackage);		
			
			if(Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + ": package " + myPackage.getPackageID() + " added");
		}
	}
	
	/**
	 * give into about current package count
	 * 
     * @author Matthias
     */
	private class GetPackageCountBehaviour extends CyclicReceiverBehaviour {
		protected GetPackageCountBehaviour(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) {
			PackageAgent currentAgent = (PackageAgent)myAgent;
			
			if(Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " <- GET_PACKAGE_COUNT");
			
			ACLMessage msgReply = new ACLMessage(MessageType.GET_PACKAGE_COUNT);
			msgReply.addUserDefinedParameter("package_count", "" + currentAgent.lstPackage.size());
			msgReply.addReceiver(msg.getSender());
			
			if(Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + " -> GET_PACKAGE_COUNT");
			
			send(msgReply);
		}
	}
	
	/**
	 * remove package from agent
	 * 
     * @author Matthias
     */
	private class RemovePackageBehaviour extends CyclicReceiverBehaviour {
		public RemovePackageBehaviour(MessageTemplate mt) {
			super(mt);
		}

		public void onMessage(ACLMessage msg) throws UnreadableException {
			PackageAgent currentAgent = (PackageAgent)myAgent;
			
			PackageData myPackage = (PackageData)msg.getContentObject();		
			currentAgent.lstPackage.remove(myPackage);
			
			if(Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + ": package " + myPackage.getPackageID() + " removed");
		}
	}
}
