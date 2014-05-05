package uni.oldenburg.server.agent;

import java.io.IOException;

import jade.core.AID;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.behaviour.TimeoutReceiverBehaviour;
import uni.oldenburg.server.agent.data.EnquiredPackageData;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("serial")
public class RampOrderAgent extends Agent {
	public final static String NAME = "RampOrderAgent";

	private int conveyorID = 0;
	private int szenarioID = 0;
	private int rampType = -1; //-1 represents a Vehicle
	private Logger logger = Logger.getLogger(RampOrderAgent.class);

	/**
	 * @author Matthias, siajkubo
	 */
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

      if (rampType == ConveyorRamp.RAMP_ENTRANCE) {
         //Eingang -> Ausgang: Schreibe Ausgangsrampen an und Frage ob das Paket dort ankommen darf
         addBehaviour(new EnquireOutgoingRampsForPackageSlotBehaviour(MessageType.START_RAMP_SEARCH_FOR_PACKAGE));

         //Eingang -> Ausgang: wähle eine Ausgangsrampe aus und melde dieser RESERVE_PACKAGE_SLOT_ON_RAMP
         addBehaviour(new SelectDestinationRampBehaviour(this,
               MessageTemplate.or(MessageTemplate.MatchPerformative(MessageType.START_RAMP_EXIT_PACKAGE_ENQUIRE),
                     MessageTemplate.MatchPerformative(MessageType.PACKAGE_IS_NEEDED_FROM_RAMP_EXIT))));

         //Eingang -> PackageAgent: setze Destination
         addBehaviour(new AssignPackageDestinationBehaviour(MessageType.ASSIGN_PACKAGE_DESTINATION));
      }

		if (rampType == ConveyorRamp.RAMP_EXIT) {

			addBehaviour(new AskOtherOrderagentsIfPackageExistsBehaviour(MessageTemplate.MatchPerformative(MessageType.SEARCH_FOR_PACKAGE)));
			addBehaviour(new SetPackageReservedBehaviour(MessageTemplate.MatchPerformative(MessageType.GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT)));
			
		}

		if (rampType == ConveyorRamp.RAMP_STOREAGE) {
			addBehaviour(new CheckIfPackageIsStoredBehaviour(MessageTemplate.MatchPerformative(MessageType.ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS)));

			addBehaviour(new AskOtherOrderagentsIfPackageExistsBehaviour(
					MessageTemplate.MatchPerformative(MessageType.SEARCH_FOR_PACKAGE)));
			addBehaviour(new SetPackageReservedBehaviour(
					MessageTemplate.MatchPerformative(MessageType.GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT)));
			addBehaviour(new CheckIfPackageIsStoredBehaviour(
					MessageTemplate.MatchPerformative(MessageType.ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS)));

         //Ausgang -> Eingang: Beantworte die Anfrage, wenn das Paket angenommen werden kann
         addBehaviour(new HandlePackageSlotEnquire(MessageType.ENQUIRE_OUTGOING_RAMP));
         //PackageAgent -> Ausgang: Der eigene PackageAgent meldet auf Anfrage eine Nachfrage nach dem Paket
         addBehaviour(new SendPackageNeedOffer(MessageType.PACKAGE_IS_NEEDED));
         //Ausgang -> Eingang: reserviere einen Slot für das Paket
         addBehaviour(new HandlePackageSlotReservationBehaviour(MessageType.RESERVE_PACKAGE_SLOT_ON_RAMP));
      }

		

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
				logger.log(Level.INFO, myAgent.getLocalName()+ " <- SEARCH_FOR_PACKAGE");

			// Receive the Request from the Packageagent
			searchedPackage = (PackageData) msg.getContentObject();

			if (Debugging.showInfoMessages)
				logger.log(Level.INFO, myAgent.getLocalName()+ " ->ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS");

			// Send the Request to all Orderagents from Storage
			ACLMessage msgPackage = new ACLMessage(MessageType.ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS);
			msgPackage.setContentObject(searchedPackage);
			AgentHelper.addReceivers(msgPackage, currentAgent, currentAgent.getSzenarioID());
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
			if (Debugging.showInfoMessages)logger.log(Level.INFO, myAgent.getLocalName()+ " -> CHECK_IF_PACKAGE_IS_STORED");

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
			if (msgAnswer.getUserDefinedParameter("answer_if_contained") != null && msgAnswer.getUserDefinedParameter("answer_if_contained")
							.equals("Yes")) {
				// Inform the Exit
				ACLMessage msgAnswerExit = new ACLMessage(
						MessageType.GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT);
				msgAnswerExit.setContentObject(msgAnswer.getContentObject());
				msgAnswerExit.addReceiver(msg.getSender());
				if (Debugging.showInfoMessages)
		            logger.log(Level.INFO, myAgent.getLocalName()+ " -> GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT");
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
					logger.log(Level.INFO, myAgent.getLocalName()+ " -> SET_PACKAGE_RESERVED");
                send(msgSetPackage);
			}

		}

	}

	public int getSzenarioID() {
		// TODO Auto-generated method stub
		return this.szenarioID;
	}

   /**
    * Enquire all ExitRamps if they have a need for an specific Package.
    *
    * @author sijakubo
    */
   private class EnquireOutgoingRampsForPackageSlotBehaviour extends CyclicReceiverBehaviour {
      protected EnquireOutgoingRampsForPackageSlotBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         PackageData packageData = (PackageData) msg.getContentObject();

         EnquiredPackageData enquiredPackageData = new EnquiredPackageData(myAgent.getAID(), packageData);

         //send enquire to exitRamps
         ACLMessage msgEnquireExitRamps = new ACLMessage(MessageType.ENQUIRE_OUTGOING_RAMP);
         msgEnquireExitRamps.setContentObject(enquiredPackageData);
         AgentHelper.addReceiver(msgEnquireExitRamps, myAgent, RampOrderAgent.NAME, conveyorID, szenarioID);
         send(msgEnquireExitRamps);

         //notify this OrderAgent, that the Enquire started
         ACLMessage msgStartExitRampEnquire = new ACLMessage(MessageType.START_RAMP_EXIT_PACKAGE_ENQUIRE);
         msgStartExitRampEnquire.addReceiver(myAgent.getAID());
         send(msgStartExitRampEnquire);
      }
   }


   /**
    * Behaviour which asks its PackageAgent if there is a need for a specifig Package
    *
    * @author sijakubo
    */
   private class HandlePackageSlotEnquire extends CyclicReceiverBehaviour {
      protected HandlePackageSlotEnquire(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         EnquiredPackageData enquiredPackage = (EnquiredPackageData) msg.getContentObject();

         ACLMessage msgCheckIfPackageIsNeeded = new ACLMessage(MessageType.CHECK_IF_PACKAGE_IS_NEEDED);
         msgCheckIfPackageIsNeeded.setContentObject(enquiredPackage);
         AgentHelper.addReceiver(msgCheckIfPackageIsNeeded, myAgent, PackageAgent.NAME, conveyorID, szenarioID);
         send(msgCheckIfPackageIsNeeded);
      }
   }

   /**
    * TimeoutReceiverBehaviour which waits for 5s to receive
    *
    * @author sijakubo
    */
   private class SelectDestinationRampBehaviour extends TimeoutReceiverBehaviour {
      public static final int TIMEOUT_MS = 5000;

      private List<AID> possibleDestinationRamps;

      public SelectDestinationRampBehaviour(Agent myAgent, MessageTemplate messageTemplate) {
         super(myAgent, TIMEOUT_MS, messageTemplate);
         possibleDestinationRamps = new ArrayList<AID>();
      }

      @Override
      public void onMessage(ACLMessage msg) {
         //Offer from ExitRamp received. Cache Recevier information
         possibleDestinationRamps.add(msg.getSender());
      }

      @Override
      public void onTimeout() throws IOException {
         if (!possibleDestinationRamps.isEmpty()) {

            AID destinationRampAID = possibleDestinationRamps.get(0);
            ACLMessage message = new ACLMessage(MessageType.RESERVE_PACKAGE_SLOT_ON_RAMP);
            message.addReceiver(destinationRampAID);
            send(message);

            possibleDestinationRamps.clear();
         } else {
            //no ExitRamp has a need for the given package. Search for a storageRamp instead
            //TODO: wir hahben aktuel keinen identifikator für Paketentypen. Wonach soll eine passende Zwischenrampe gesucht werden?
         }
      }

   }

   /**
    * EntranceRamp -> ExitRamp
    *
    * The ExitRamp got choosen to receiver the Package. Reserve the Job on the PackageAgent
    *
    * @author sijakubo
    */
   private class HandlePackageSlotReservationBehaviour extends CyclicReceiverBehaviour {

      protected HandlePackageSlotReservationBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         PackageData packageData = (PackageData) msg.getContentObject();

         //Notifiy PackageAgent, to reserve the Slot for the Package
         ACLMessage msgAcceptReservationOffer = new ACLMessage(MessageType.SET_PACKAGE_RESERVED);
         msgAcceptReservationOffer.setContentObject(packageData);
         AgentHelper.addReceiver(msgAcceptReservationOffer, myAgent, PackageAgent.NAME, conveyorID, szenarioID);
         send(msgAcceptReservationOffer);

         ACLMessage msgReply = new ACLMessage(MessageType.ASSIGN_PACKAGE_DESTINATION);
         msgReply.setContentObject(packageData);
         msgReply.addReceiver(msg.getSender());
         send(msgReply);
      }
   }

   /**
    * @author sijakubo
    */
   private class AssignPackageDestinationBehaviour extends CyclicReceiverBehaviour {

      protected AssignPackageDestinationBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         PackageData packageData = (PackageData) msg.getContentObject();

         ACLMessage msgAssignDestination = new ACLMessage(MessageType.ASSIGN_PACKAGE_DESTINATION);
         msgAssignDestination.setContentObject(packageData.getPackageID());
         AgentHelper.addReceiver(msgAssignDestination, myAgent, PackageAgent.NAME, conveyorID, szenarioID);
      }
   }

   /**
    * @author sijakubo
    */
   private class SendPackageNeedOffer extends CyclicReceiverBehaviour {

      protected SendPackageNeedOffer(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         EnquiredPackageData enquiredPackageData = (EnquiredPackageData) msg.getContentObject();

         ACLMessage msgPackageIsNeeded = new ACLMessage(MessageType.PACKAGE_IS_NEEDED_FROM_RAMP_EXIT);
         msgPackageIsNeeded.addReceiver(enquiredPackageData.getEnquirererAID());
         msgPackageIsNeeded.setContentObject(enquiredPackageData.getPackageData());
         send(msgPackageIsNeeded);
      }
   }
}
