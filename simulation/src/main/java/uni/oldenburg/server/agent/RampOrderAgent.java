package uni.oldenburg.server.agent;

import java.io.IOException;

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
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

@SuppressWarnings("serial")
public class RampOrderAgent extends Agent {
	public final static String NAME = "RampOrderAgent";

	private int conveyorID = 0;
	private int szenarioID = 0;
	private int rampType = -1; //-1 represents a Vehicle
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
			if (myConveyor instanceof ConveyorRamp) {
				rampType = ((ConveyorRamp) myConveyor).getRampType();
			}
		}

		if (rampType == ConveyorRamp.RAMP_EXIT) {
			addBehaviour(new AskOtherOrderagentsIfPackageExistsBehaviour(
					MessageTemplate
							.MatchPerformative(MessageType.SEARCH_FOR_PACKAGE)));
			addBehaviour(new SetPackageReservedBehaviour(
					MessageTemplate
							.MatchPerformative(MessageType.GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT)));
			
		}

		if (rampType == ConveyorRamp.RAMP_STOREAGE) {
			addBehaviour(new CheckIfPackageIsStoredBehaviour(
					MessageTemplate
							.MatchPerformative(MessageType.ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS)));
			
		}

		// addBehaviour(new AssignDestinationBehaviour());
		// addBehaviour(new AssignVehicleForPackageBehaviour());
		// addBehaviour(new CanTakePackageBehaviour());
		// addBehaviour(new ExistsDestinationBehaviour());
		// addBehaviour(new InitializePacketAgentBehaviour());
		// addBehaviour(new IsCapacityAvailableBehaviour());
		// addBehaviour(new NotifyPacketArrivalBehaviour());
		// addBehaviour(new ReserveSpaceBehaviour());

		String nickname = AgentHelper.getUniqueNickname(RampOrderAgent.NAME,
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
	 * Behaviour should receive the request of a Packageagent and should ask the
	 * Orderagents of a Storage if there is a Package for the given ID
	 * 
	 * @author Raschid
	 */
	private class AskOtherOrderagentsIfPackageExistsBehaviour extends
			CyclicReceiverBehaviour {

		protected AskOtherOrderagentsIfPackageExistsBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			RampOrderAgent currentAgent = (RampOrderAgent) myAgent;
			PackageData searchedPackage = null;

			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " <- SEARCH_FOR_PACKAGE");

			// Receive the Request from the Packageagent
			searchedPackage = (PackageData) msg.getContentObject();

			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " ->ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS");

			// Send the Request to all Orderagents from Storage
			ACLMessage msgPackage = new ACLMessage(
					MessageType.ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS);
			msgPackage.setContentObject(searchedPackage);
			AgentHelper.addReceivers(msgPackage, currentAgent,
					currentAgent.getSzenarioID());
			send(msgPackage);

		}

	}

	/**
	 * Behaviour should receive the request from an exit Orderagent and should
	 * ask the Packageagent if a package is stored, which is needed by the exit
	 * and then answer the Orderagent of the exit if the Package exists or not
	 * 
	 * @author Raschid
	 */
	private class CheckIfPackageIsStoredBehaviour extends
			CyclicReceiverBehaviour {

		protected CheckIfPackageIsStoredBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			RampOrderAgent currentAgent = (RampOrderAgent) myAgent;
			PackageData searchedPackage = null;

			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " <- ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS");

			// Receive the Message from the Exit
			searchedPackage = (PackageData) msg.getContentObject();

			// Orderagent should ask his Packageagent if the Package is stored
			ACLMessage msgCheckPackage = new ACLMessage(
					MessageType.CHECK_IF_PACKAGE_IS_STORED);
			msgCheckPackage.setContentObject(searchedPackage);
			AgentHelper.addReceiver(msgCheckPackage, currentAgent,
					PackageAgent.NAME, conveyorID, szenarioID);
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " -> CHECK_IF_PACKAGE_IS_STORED");

			send(msgCheckPackage);

			// Get the answer from Packageagent
			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " <- ANSWER_IF_PACKAGE_IS_CONTAINED");
			MessageTemplate mt = MessageTemplate
					.MatchPerformative(MessageType.ANSWER_IF_PACKAGE_IS_CONTAINED);
			ACLMessage msgAnswer = currentAgent.blockingReceive(mt);
			// If it is answered with Yes, then he should inform the Exit and
			// his Routingagent to start an Auction
			if (msgAnswer.getUserDefinedParameter("answer_if_contained") != null
					&& msgAnswer.getUserDefinedParameter("answer_if_contained")
							.equals("Yes")) {
				// Inform the Exit
				ACLMessage msgAnswerExit = new ACLMessage(
						MessageType.GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT);
				msgAnswerExit.setContentObject(msgAnswer.getContentObject());
				msgAnswerExit.addReceiver(msg.getSender());
				if (Debugging.showInfoMessages)
					logger.log(
							Level.INFO,
							myAgent.getLocalName()
									+ " -> GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT");
				send(msgAnswer);

				// Inform the Routingagent

			}
			/*
			 * else { //Inform the Exit ACLMessage msgAnswerExit = new
			 * ACLMessage(MessageType.GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT);
			 * msgAnswerExit.addUserDefinedParameter("answer_exit", "No");
			 * msgAnswerExit.addReceiver(msg.getSender());
			 * if(Debugging.showInfoMessages) logger.log(Level.INFO,
			 * myAgent.getLocalName() +
			 * " ->Answer Exit  With No if Package is stored"); send(msgAnswer);
			 * 
			 * }
			 */

		}

	}

	/**
	 * Behaviour should receive the answer from the Storage and should set the
	 * Packagedata reserved
	 * 
	 * @author Raschid
	 */
	private class SetPackageReservedBehaviour extends CyclicReceiverBehaviour {

		protected SetPackageReservedBehaviour(MessageTemplate mt) {
			super(mt);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void onMessage(ACLMessage msg) throws UnreadableException,
				IOException {
			RampOrderAgent currentAgent = (RampOrderAgent) myAgent;
			PackageData searchedPackage = null;

			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()
						+ " <- GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT");
			
			searchedPackage=(PackageData) msg.getContentObject();
			
			//Tell the Packageagent to set the Package reserved, so that it will not be checked again
			if(searchedPackage!=null){
				// Orderagent should ask his Packageagent to set a Package reserved
				ACLMessage msgSetPackage = new ACLMessage(
						MessageType.SET_PACKAGE_RESERVED);
				msgSetPackage.setContentObject(searchedPackage);
				AgentHelper.addReceiver(msgSetPackage, currentAgent,
						PackageAgent.NAME, conveyorID, szenarioID);
				if (Debugging.showInfoMessages)
					logger.log(Level.INFO, myAgent.getLocalName()
							+ " -> SET_PACKAGE_RESERVED");

				send(msgSetPackage);
			}

		}

	}

	public int getSzenarioID() {
		// TODO Auto-generated method stub
		return this.szenarioID;
	}

}
