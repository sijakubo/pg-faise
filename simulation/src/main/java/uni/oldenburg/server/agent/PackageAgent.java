package uni.oldenburg.server.agent;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.TickerBehaviour;
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
	private int rampType = 0;

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
			rampType = ((ConveyorRamp) myConveyor).getRampType();

		}

		addBehaviour(new AddPackageBehaviour(
				MessageTemplate.MatchPerformative(MessageType.ADD_PACKAGE)));
		addBehaviour(new GetPackageCountBehaviour(
				MessageTemplate
						.MatchPerformative(MessageType.GET_PACKAGE_COUNT)));
		addBehaviour(new RemovePackageBehaviour(
				MessageTemplate.MatchPerformative(MessageType.REMOVE_PACKAGE)));

		// If it is an Exit, than add the Behaviour, which is used for requesting an Package for an existing Job
		if (rampType == 2) {
          addBehaviour(new SearchForPackageBehaviour(this,3000));
		}

		String nickname = AgentHelper.getUniqueNickname(PackageAgent.NAME,
				conveyorID, szenarioID);
		AgentHelper.registerAgent(szenarioID, this, nickname);

		if (Debugging.showAgentStartupMessages)
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
			PackageAgent currentAgent = (PackageAgent) myAgent;
			PackageData myPackage = (PackageData) msg.getContentObject();

			currentAgent.lstPackage.add(myPackage);

			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + ": package "
						+ myPackage.getPackageID() + " added");
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
			PackageAgent currentAgent = (PackageAgent) myAgent;

			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " <- GET_PACKAGE_COUNT");

			ACLMessage msgReply = new ACLMessage(MessageType.GET_PACKAGE_COUNT);
			msgReply.addUserDefinedParameter("package_count", ""
					+ currentAgent.lstPackage.size());
			msgReply.addReceiver(msg.getSender());

			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " -> GET_PACKAGE_COUNT");

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
			PackageAgent currentAgent = (PackageAgent) myAgent;

			PackageData myPackage = (PackageData) msg.getContentObject();
			currentAgent.lstPackage.remove(myPackage);

			if (Debugging.showPackageMessages)
				logger.log(Level.INFO, myAgent.getLocalName() + ": package "
						+ myPackage.getPackageID() + " removed");
		}
	}

	/**
	 * Behaviour should check cyclically if the Storage (Zwischenlager) has a
	 * Package for which a Job exists at the Exit
	 * 
	 * @author Raschid
	 */
	private class SearchForPackageBehaviour extends TickerBehaviour {

		public SearchForPackageBehaviour(Agent a, long period) {
			super(a, period);

		}

		@Override
		protected void onTick() {
			PackageAgent currentAgent = (PackageAgent) myAgent;

			// Only if Jobs exists, there should be made a request
			int sizePackageList = currentAgent.lstPackage.size();
			if (sizePackageList >= 1) {
				// Choose a Package randomly. To do that, you have to create a
				// random Value depending on the size of the package list
				int index = -1;
				if (sizePackageList == 1) {
					index = 0;
				} else if (sizePackageList == 2) {
					Random random = new Random();
					index = random.nextInt(1 - 0 + 1) + 0;
				} else if (sizePackageList == 3) {
					Random random = new Random();
					index = random.nextInt(2 - 1 + 1) + 1;
				} else if (sizePackageList == 4) {
					Random random = new Random();
					index = random.nextInt(3 - 2 + 1) + 2;
				}

				// Get the Package Data
				PackageData pData = currentAgent.lstPackage.get(index);

				// Send the Message
				ACLMessage msgPackageSearch = new ACLMessage(
						MessageType.SEARCH_FOR_PACKAGE);
				AgentHelper.addReceiver(msgPackageSearch, myAgent,
						RampOrderAgent.NAME, currentAgent.conveyorID,
						currentAgent.szenarioID);
				try {
					msgPackageSearch.setContentObject(pData);
					send(msgPackageSearch);
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			}

		}

	}
	
	/**
	 * Behaviour should receive the request from its Storage Orderagent and should search the list and look if the requested Package exists
	 * and then answer the Orderagent
	 * @author Raschid
	 */
	private class CheckIfPackageIsContainedBehaviour extends CyclicBehaviour {

		@Override
		public void action() {
			
			
		}	
		
	}

}
