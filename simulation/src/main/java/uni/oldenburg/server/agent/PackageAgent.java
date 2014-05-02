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
	private int rampType = -1;

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
			if (myConveyor instanceof ConveyorRamp) {
				rampType = ((ConveyorRamp) myConveyor).getRampType();
			}

		}

		addBehaviour(new AddPackageBehaviour(
				MessageTemplate.MatchPerformative(MessageType.ADD_PACKAGE)));
		addBehaviour(new GetPackageCountBehaviour(
				MessageTemplate
						.MatchPerformative(MessageType.GET_PACKAGE_COUNT)));
		addBehaviour(new RemovePackageBehaviour(
				MessageTemplate.MatchPerformative(MessageType.REMOVE_PACKAGE)));

		// If it is an Exit, than add the Behaviour, which is used for
		// requesting an Package for an existing Job
		if (rampType == ConveyorRamp.RAMP_EXIT) {
			addBehaviour(new SearchForPackageBehaviour(this, 3000));
		}

		// If it is an Storage it should answer the request from its own
		// Orderagent, who was asked by an exit, and check if a
		// Package for the requested Package id is contained
		if (rampType == ConveyorRamp.RAMP_STOREAGE) {
			addBehaviour(new AnswerIfPackageIsContainedBehaviour(
					MessageTemplate
							.MatchPerformative(MessageType.CHECK_IF_PACKAGE_IS_STORED)));
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

				// Send the Message, if the Package is not reserved
				if (!pData.isReserved()) {
					ACLMessage msgPackageSearch = new ACLMessage(
							MessageType.SEARCH_FOR_PACKAGE);
					AgentHelper.addReceiver(msgPackageSearch, myAgent,
							RampOrderAgent.NAME, currentAgent.conveyorID,
							currentAgent.szenarioID);

					if (Debugging.showInfoMessages)
						logger.log(Level.INFO, myAgent.getLocalName()
								+ " -> SEARCH_FOR_PACKAGE");

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

	}

	/**
	 * Behaviour should receive the request from its Storage Orderagent and
	 * should search the list and look if the requested Package exists and then
	 * answer the Orderagent
	 * 
	 * @author Raschid
	 */
	private class AnswerIfPackageIsContainedBehaviour extends
			CyclicReceiverBehaviour {

		protected AnswerIfPackageIsContainedBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			// Receive the Request from Orderagent
			PackageAgent currentAgent = (PackageAgent) myAgent;
			PackageData searchedPackage = null;

			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " -> RECEIVE_REQUEST_FROM_PACKAGEAGENT");

			searchedPackage = (PackageData) msg.getContentObject();
			// get the Package id
			int id = searchedPackage.getPackageID();

			// Check if it is contained at the first position
			int size = currentAgent.lstPackage.size();
			if (size >= 1) {
				PackageData firstPositionPackage = currentAgent.lstPackage
						.get(size - 1);
				if (id == firstPositionPackage.getPackageID()) {
					// Answer the Orderagent with Yes
					ACLMessage msgAnswer = new ACLMessage(
							MessageType.ANSWER_IF_PACKAGE_IS_CONTAINED);
					msgAnswer.addUserDefinedParameter("answer_if_contained",
							"Yes");
					msgAnswer.setContentObject(searchedPackage);
					msgAnswer.addReceiver(msg.getSender());
					send(msgAnswer);

				} else {
					// Answer the Orderagent with No
					ACLMessage msgAnswer = new ACLMessage(
							MessageType.ANSWER_IF_PACKAGE_IS_CONTAINED);
					msgAnswer.addUserDefinedParameter("answer_if_contained",
							"No");
					msgAnswer.setContentObject(searchedPackage);
					msgAnswer.addReceiver(msg.getSender());
					send(msgAnswer);

				}
			} else {
				// Answer the Orderagent with No if there is no Package in the
				// list
				ACLMessage msgAnswer = new ACLMessage(
						MessageType.ANSWER_IF_PACKAGE_IS_CONTAINED);
				msgAnswer.addUserDefinedParameter("answer_if_contained", "No");
				msgAnswer.setContentObject(searchedPackage);
				msgAnswer.addReceiver(msg.getSender());
				send(msgAnswer);
			}

		}

	}

}
