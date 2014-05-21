package uni.oldenburg.server.agent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.WakerBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import uni.oldenburg.Debugging;
import uni.oldenburg.server.agent.behaviour.CyclicReceiverBehaviour;
import uni.oldenburg.server.agent.data.PackageData;
import uni.oldenburg.server.agent.helper.AgentHelper;
import uni.oldenburg.server.agent.message.MessageType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("serial")
public class RampOrderAgent extends Agent {
   public final static String NAME = "RampOrderAgent";

   private static final String ENQUIRING_RAMP_PARAMETER_KEY = "enquiring_ramp_conveyor_id";

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
         //Eingang -> Ausgang: wähle eine Ausgangsrampe aus und melde dieser RESERVE_PACKAGE_SLOT_ON_RAMP
         addBehaviour(new EnquireRampsForPackageSlotBehaviour(
               MessageType.START_EXIT_RAMP_SEARCH_FOR_PACKAGE,
               MessageType.END_EXIT_RAMP_SEARCH_FOR_PACKAGE,
               MessageType.ENQUIRE_EXIT_RAMP,
               MessageType.PACKAGE_IS_NEEDED_FROM_EXIT_RAMP));

         //Eingang -> Zwischenrampe: wähle eine Zwischenrampe aus und melde dieser RESERVE_PACKAGE_SLOT_ON_RAMP
         addBehaviour(new EnquireRampsForPackageSlotBehaviour(
               MessageType.START_STORAGE_RAMP_SEARCH_FOR_PACKAGE,
               MessageType.END_STORAGE_RAMP_SEARCH_FOR_PACKAGE,
               MessageType.ENQUIRE_STORAGE_RAMP,
               MessageType.PACKAGE_IS_STORABLE_FROM_STORAGE_RAMP));

         //Eingang -> PackageAgent: setze Destination
         addBehaviour(new AssignPackageDestinationBehaviour(MessageType.ASSIGN_PACKAGE_DESTINATION));
      }

      if (rampType == ConveyorRamp.RAMP_EXIT) {
         addBehaviour(new AskOtherOrderagentsIfPackageExistsBehaviour(MessageTemplate.MatchPerformative(MessageType.SEARCH_FOR_PACKAGE)));
         addBehaviour(new SetPackageReservedBehaviour(MessageTemplate.MatchPerformative(MessageType.GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT)));

         //Ausgang -> Eingang: Beantworte die Anfrage, wenn das Paket angenommen werden kann
         addBehaviour(new HandleExitRampPackageSlotEnquireBehaviour(MessageType.ENQUIRE_EXIT_RAMP));
         //PackageAgent -> Ausgang: Der eigene PackageAgent meldet auf Anfrage eine Nachfrage nach dem Paket
         addBehaviour(new SendPackageNeedOfferBehaviour(MessageType.PACKAGE_IS_NEEDED));
         //Ausgang -> Eingang: reserviere einen Slot für das Paket
         addBehaviour(new HandlePackageSlotReservationBehaviour(MessageType.RESERVE_PACKAGE_SLOT_ON_RAMP, false));
      }

      if (rampType == ConveyorRamp.RAMP_STOREAGE) {
         addBehaviour(new CheckIfPackageIsStoredBehaviour(MessageTemplate.MatchPerformative(MessageType.ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS)));

         addBehaviour(new AskOtherOrderagentsIfPackageExistsBehaviour(
               MessageTemplate.MatchPerformative(MessageType.SEARCH_FOR_PACKAGE)));
         addBehaviour(new SetPackageReservedBehaviour(
               MessageTemplate.MatchPerformative(MessageType.GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT)));
         addBehaviour(new CheckIfPackageIsStoredBehaviour(
               MessageTemplate.MatchPerformative(MessageType.ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS)));

         addBehaviour(new HandleStorageRampPackageSlotEnquireBehaviour(MessageType.ENQUIRE_STORAGE_RAMP));
         addBehaviour(new HandlePackageSlotReservationBehaviour(MessageType.RESERVE_PACKAGE_SLOT_ON_RAMP, true));
         addBehaviour(new HandleSpaceAvailableBehaviour(MessageType.PACKAGE_SPACE_AVAILABLE));
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
    * Got message:
    * Packageagent: SearchForPackageBehaviour
    * Send message:
    * RampOrderagent: CheckIfPackageIsStoredBehaviour
    * Behaviour should receive the request of a Packageagent and should ask the
    * Orderagents of a Storage if there is a Package for the given ID
    *
    * @author Raschid
    */
   private class AskOtherOrderagentsIfPackageExistsBehaviour extends
         CyclicReceiverBehaviour {

      protected AskOtherOrderagentsIfPackageExistsBehaviour(MessageTemplate mt) {
         super(mt);
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException,
            IOException {
         RampOrderAgent currentAgent = (RampOrderAgent) myAgent;
         PackageData searchedPackage = null;

         if (Debugging.showInfoMessages)
            logger.log(Level.INFO, myAgent.getLocalName() + " <- SEARCH_FOR_PACKAGE");

         // Receive the Request from the Packageagent
         searchedPackage = (PackageData) msg.getContentObject();

         if (Debugging.showInfoMessages)
            logger.log(Level.INFO, myAgent.getLocalName() + " ->ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS");

         // Send the Request to all Orderagents from Storage
         ACLMessage msgPackage = new ACLMessage(MessageType.ASK_OTHER_ORDERAGENTS_IF_PACKAGE_EXISTS);
         msgPackage.addUserDefinedParameter("conveyorId", "" + currentAgent.conveyorID);//Send the Conveyor id, so that the Storage knows for which destination an auction should be started
         msgPackage.setContentObject(searchedPackage);
         AgentHelper.addReceivers(msgPackage, currentAgent, currentAgent.getSzenarioID());
         send(msgPackage);

      }

   }

   /**
    * Got message:
    * RampOrderagent: AskOtherOrderagentsIfPackageExistsBehaviour
    * Packageagent: AnswerIfPackageIsContainedBehaviour
    * Send message:
    * Packageagent: AnswerIfPackageIsContainedBehaviour
    * Packageagent: PackageReservationBehaviour
    * <p/>
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
         AgentHelper.addReceiver(msgCheckPackage, currentAgent, PackageAgent.NAME, conveyorID, szenarioID);
         if (Debugging.showInfoMessages)
            logger.log(Level.INFO, myAgent.getLocalName() + " -> CHECK_IF_PACKAGE_IS_STORED");

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
            ACLMessage msgAnswerExit = new ACLMessage(MessageType.GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT);

            msgAnswerExit.setContentObject(msgAnswer.getContentObject());
            msgAnswerExit.addReceiver(msg.getSender());
            if (Debugging.showInfoMessages)
               logger.log(Level.INFO, myAgent.getLocalName() + " -> GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT");
            send(msgAnswer);


            // Inform the PackageAgent of the Storage to set the Package as reserved and to set the Destination ID
            ACLMessage msgAnswerPackageAgent = new ACLMessage(MessageType.SET_PACKAGE_RESERVED);
            msgAnswerPackageAgent.addUserDefinedParameter("conveyorId", msg.getUserDefinedParameter("conveyorId"));
            msgAnswerPackageAgent.setContentObject(msgAnswer.getContentObject());
            AgentHelper.addReceiver(msgAnswerPackageAgent, currentAgent, PackageAgent.NAME, conveyorID, szenarioID);
            if (Debugging.showInfoMessages)
               logger.log(Level.INFO, myAgent.getLocalName() + " -> SET_PACKAGE_RESERVED");
            send(msgAnswer);

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
    * Got message:
    * RampOrderagent: CheckIfPackageIsStoredBehaviour
    * Send message:
    * Packageagent: PackageReservationBehaviour
    * Behaviour should receive the answer from the Storage and should set the
    * Packagedata reserved
    *
    * @author Raschid
    */
   private class SetPackageReservedBehaviour extends CyclicReceiverBehaviour {

      protected SetPackageReservedBehaviour(MessageTemplate mt) {
         super(mt);
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException,
            IOException {
         RampOrderAgent currentAgent = (RampOrderAgent) myAgent;
         PackageData searchedPackage = null;

         if (Debugging.showInfoMessages)
            logger.log(Level.INFO, myAgent.getLocalName()
                  + " <- GET_ANSWER_IF_PACKAGE_IS_STORED_OR_NOT");

         searchedPackage = (PackageData) msg.getContentObject();

         //Tell the Packageagent to set the Package reserved, so that it will not be checked again
         if (searchedPackage != null) {
            // Orderagent should ask his Packageagent to set a Package reserved
            ACLMessage msgSetPackage = new ACLMessage(
                  MessageType.SET_PACKAGE_RESERVED);
            msgSetPackage.setContentObject(searchedPackage);
            AgentHelper.addReceiver(msgSetPackage, currentAgent,
                  PackageAgent.NAME, conveyorID, szenarioID);
            if (Debugging.showInfoMessages)
               logger.log(Level.INFO, myAgent.getLocalName() + " -> SET_PACKAGE_RESERVED");
            send(msgSetPackage);
         }

      }

   }

   public int getSzenarioID() {
      return this.szenarioID;
   }

   /**
    * Behaviour on:
    * Entrance - RampOrderAgent
    * <p/>
    * Gets message:
    * PackageAgent:  START_EXIT_RAMP_SEARCH_FOR_PACKAGE / START_STORAGE_RAMP_SEARCH_FOR_PACKAGE
    * Entrance - RampOrderAgent: END_EXIT_RAMP_SEARCH_FOR_PACKAGE / END_STORAGE_RAMP_SEARCH_FOR_PACKAGE
    * <p/>
    * Send message:
    * Exit or Storage - RampOrderAgent: ENQUIRE_EXIT_RAMP / ENQUIRE_STORAGE_RAMP
    * Exit or Storage - RampOrderAgent: RESERVE_PACKAGE_SLOT_ON_RAMP
    * <p/>
    * Behaviour that handles the Search for an Destination Ramp. This Behaviour should receive a request from the
    * PlattformAgent when a new package arrives which needs to be distributed.
    * <p/>
    * This Behaviour could either ask Exit- or StorageRamps.
    * If no free Conveyor was found, this behaviour asks in 5s steps the StorageRamps until a free space is found.
    *
    * @author sijakubo
    */
   private class EnquireRampsForPackageSlotBehaviour extends CyclicReceiverBehaviour {
      private static final int ENQUIRE_TIMEOUT = 5000;
      private Behaviour timeOutBehaviour;
      private List<AID> possibleDestinationRamps;
      private int startEnquireMessageType;
      private int endEnquireMessageType;
      private final int enquireMessageType;
      private int receiveSlotOfferMessageType;

      protected EnquireRampsForPackageSlotBehaviour(int startEnquireMessageType,
                                                    int endEnquireMessageType,
                                                    int enquireMessageType,
                                                    int receiveSlotOfferMessageType) {
         super(MessageTemplate.or(
                  MessageTemplate.or(
                        MessageTemplate.MatchPerformative(startEnquireMessageType),
                        MessageTemplate.MatchPerformative(receiveSlotOfferMessageType)
                  ),
               MessageTemplate.MatchPerformative(endEnquireMessageType)
         ));

         this.startEnquireMessageType = startEnquireMessageType;
         this.endEnquireMessageType = endEnquireMessageType;
         this.enquireMessageType = enquireMessageType;
         this.receiveSlotOfferMessageType = receiveSlotOfferMessageType;
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         PackageData packageData = (PackageData) msg.getContentObject();

         if (msg.getPerformative() == startEnquireMessageType) {
            //Start Enquire of Ramps
            logger.info("Entrance - RampOrderAgent <- START_RAMP_SEARCH_FOR_PACKAGE, from " + conveyorID);

            addTimeOutBehaviourForPackageDataToMyAgent(packageData);
            possibleDestinationRamps = new ArrayList<AID>();

            //send enquire to ramps
            ACLMessage msgEnquireRamps = new ACLMessage(enquireMessageType);
            logger.info("Entrance - RampOrderAgent -> ENQUIRE_RAMP (" + enquireMessageType+ ")");
            msgEnquireRamps.setContentObject(packageData);
            msgEnquireRamps.addUserDefinedParameter(ENQUIRING_RAMP_PARAMETER_KEY, String.valueOf(conveyorID));
            AgentHelper.addReceivers(msgEnquireRamps, myAgent, szenarioID);
            send(msgEnquireRamps);

         } else if (msg.getPerformative() == endEnquireMessageType) {
            //End Enquire of Ramps
            logger.info("Entrance - RampOrderAgent <- END_RAMP_SEARCH_FOR_PACKAGE");

            if (!possibleDestinationRamps.isEmpty()) {
               AID destinationRampAID = possibleDestinationRamps.get(0);
               logger.debug("Destination Ramp found for Package: " + packageData.getPackageID()
                     + ". Destination is AID: " + destinationRampAID);

               ACLMessage message = new ACLMessage(MessageType.RESERVE_PACKAGE_SLOT_ON_RAMP);
               message.setContentObject(packageData);
               message.addReceiver(destinationRampAID);
               send(message);

               //Start bot auction
            } else {
               logger.debug("No DestinationRamp found for Package: " + packageData.getPackageID()
                     + ". Trying to find Storage Ramp.");

               //no ExitRamp has a need for the given package.
               //Initialize StorageRamp Search until a StorageRamp is found.
               ACLMessage message = new ACLMessage(MessageType.START_STORAGE_RAMP_SEARCH_FOR_PACKAGE);
               message.addUserDefinedParameter(ENQUIRING_RAMP_PARAMETER_KEY, String.valueOf(conveyorID));
               message.setContentObject(packageData);
               message.addReceiver(myAgent.getAID());

               logger.info("Entrance - RampOrderAgent -> START_STORAGE_RAMP_SEARCH_FOR_PACKAGE");
               send(message);
            }
         } else if (msg.getPerformative() == receiveSlotOfferMessageType) {
            logger.info("Entrance - RampOrderAgent <- SLOT_OFFER from Exit or StorageRamp");
            possibleDestinationRamps.add(msg.getSender());
         }
      }

      private void addTimeOutBehaviourForPackageDataToMyAgent(final PackageData packageData) {
         if (timeOutBehaviour != null) {
            myAgent.removeBehaviour(timeOutBehaviour);
         }

         //WakerBehaviour die nach ENQUIRE_TIMEOUT millisekunden eine EndeNachricht an die Behaviour
         // EnquireRampsForPackageSlotBehaviour schickt
         timeOutBehaviour = new WakerBehaviour(myAgent, ENQUIRE_TIMEOUT) {
            @Override
            protected void onWake() {
               try {
                  logger.info("WakeUp -> Entrance (END_EXIT_RAMP_SEARCH_FOR_PACKAGE)");
                  ACLMessage endEnquireMessage = new ACLMessage(endEnquireMessageType);
                  endEnquireMessage.setContentObject(packageData);
                  endEnquireMessage.addReceiver(myAgent.getAID());
                  myAgent.send(endEnquireMessage);
               } catch (IOException e) {
                  e.printStackTrace();
               }
            }
         };

         logger.info("WakeUp-Behaviour added");
         myAgent.addBehaviour(timeOutBehaviour);
      }
   }


   /**
    * Behaviour on:
    * Exit - RampOrderAgent
    * <p/>
    * Gets message:
    * Entrance - RampOrderAgent: REQUEST_OFFER_FROM_EXIT_RAMP_FOR_PACKAGE
    * <p/>
    * Send message:
    * Exit - PackageAgent - CHECK_IF_PACKAGE_IS_NEEDED
    * <p/>
    * Behaviour on ExitRamp which receives a request from the EntranceOrderAgent to asks its PackageAgent if
    * there is a need for a specific Package
    *
    * @author sijakubo
    */
   private class HandleExitRampPackageSlotEnquireBehaviour extends CyclicReceiverBehaviour {
      protected HandleExitRampPackageSlotEnquireBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         PackageData packageData = (PackageData) msg.getContentObject();
         logger.info(myAgent.getLocalName()+ "-> CHECK_IF_PACKAGE_IS_NEEDED");
         ACLMessage msgCheckIfPackageIsNeeded = new ACLMessage(MessageType.CHECK_IF_PACKAGE_IS_NEEDED);
         msgCheckIfPackageIsNeeded.addUserDefinedParameter(ENQUIRING_RAMP_PARAMETER_KEY,
               msg.getUserDefinedParameter(ENQUIRING_RAMP_PARAMETER_KEY));
         msgCheckIfPackageIsNeeded.setContentObject(packageData);
         AgentHelper.addReceiver(msgCheckIfPackageIsNeeded, myAgent, PackageAgent.NAME, conveyorID, szenarioID);
         send(msgCheckIfPackageIsNeeded);
      }
   }


   /**
    * Behaviour on:
    * Exit or Storage - RampOrderAgent
    * <p/>
    * Gets message:
    * Entrance - RampOrderAgent: RESERVE_PACKAGE_SLOT_ON_RAMP
    * <p/>
    * Send message:
    * PackageAgent - SET_PACKAGE_RESERVED / SET_PACKAGE_RESERVED_FOR_STORAGE_RAMP
    * <p/>
    * Behaviour which receives a request from an Entrance Ramp to Reserve the Job on the
    * Exit- or StorageRamps-PackageAgent
    *
    * @author sijakubo
    */
   private class HandlePackageSlotReservationBehaviour extends CyclicReceiverBehaviour {
      private boolean isStorageRamp;

      protected HandlePackageSlotReservationBehaviour(int messageType, boolean isStorageRamp) {
         super(MessageTemplate.MatchPerformative(messageType));
         this.isStorageRamp = isStorageRamp;
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         PackageData packageData = (PackageData) msg.getContentObject();

         //Notifiy PackageAgent, to reserve the Slot for the Package

         ACLMessage msgAcceptReservationOffer;
         if (isStorageRamp) {
            msgAcceptReservationOffer = new ACLMessage(MessageType.SET_PACKAGE_RESERVED);
         } else {
            msgAcceptReservationOffer = new ACLMessage(MessageType.SET_PACKAGE_RESERVED_FOR_STORAGE_RAMP);
         }
         msgAcceptReservationOffer.setContentObject(packageData);
         AgentHelper.addReceiver(msgAcceptReservationOffer, myAgent, PackageAgent.NAME, conveyorID, szenarioID);
         send(msgAcceptReservationOffer);
         logger.info(myAgent.getLocalName()+ "-> ASSIGN_PACKAGE_DESTINATION");
         //reply to the Entrance Ramp, that the Reservation was accepted
         ACLMessage msgReply = new ACLMessage(MessageType.ASSIGN_PACKAGE_DESTINATION);
         msgReply.setContentObject(packageData);
         msgReply.addUserDefinedParameter("destination_conveyor_id", String.valueOf(conveyorID));
         msgReply.addReceiver(msg.getSender());
         send(msgReply);
      }
   }

   /**
    * Behaviour on:
    * Entrance - RampOrderAgent
    * <p/>
    * Gets message:
    * Entrance - RampOrderAgent: ASSIGN_PACKAGE_DESTINATION
    * <p/>
    * Send message:
    * Entrance - PackageAgent - ASSIGN_PACKAGE_DESTINATION
    * <p/>
    * Behaviour on an EntranceRamp which sends a request to its PackageAgent to set a destination on a specific package
    *
    * @author sijakubo
    */
   private class AssignPackageDestinationBehaviour extends CyclicReceiverBehaviour {

      protected AssignPackageDestinationBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         logger.info("Entrance - RampOrderAgent -> PackageAgent.ASSIGN_PACKAGE_DESTINATION");
         PackageData packageData = (PackageData) msg.getContentObject();

         ACLMessage msgAssignDestination = new ACLMessage(MessageType.ASSIGN_PACKAGE_DESTINATION);
         msgAssignDestination.addUserDefinedParameter(
               "package_id", String.valueOf(packageData.getPackageID()));
         msgAssignDestination.addUserDefinedParameter(
               "destination_conveyor_id", msg.getUserDefinedParameter("destination_conveyor_id"));

         msgAssignDestination.setContentObject(packageData.getPackageID());
         AgentHelper.addReceiver(msgAssignDestination, myAgent, PackageAgent.NAME, conveyorID, szenarioID);
         send(msgAssignDestination);
      }
   }

   /**
    * Behaviour on:
    * Exit - RampeOrderAgent
    * <p/>
    * Gets message:
    * PackageAgent: PACKAGE_IS_NEEDED
    * <p/>
    * Send message:
    * Exit - RampOrderAgent: PACKAGE_IS_NEEDED_FROM_EXIT_RAMP
    * <p/>
    * Behaviour on an ExitRamp which send a response to an EntranceRamp package Enquire.
    *
    * @author sijakubo
    */
   private class SendPackageNeedOfferBehaviour extends CyclicReceiverBehaviour {

      protected SendPackageNeedOfferBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         ACLMessage msgPackageIsNeeded = new ACLMessage(MessageType.PACKAGE_IS_NEEDED_FROM_EXIT_RAMP);

         String enquiringRampConveyorId = msg.getUserDefinedParameter(ENQUIRING_RAMP_PARAMETER_KEY);
         logger.info("Exit - RampOrderAgent -> PACKAGE_IS_NEEDED_FROM_EXIT_RAMP");
         AgentHelper.addReceiver(msgPackageIsNeeded, myAgent, RampOrderAgent.NAME,
               Integer.valueOf(enquiringRampConveyorId), szenarioID);

         //packageData
         msgPackageIsNeeded.setContentObject(msg.getContentObject());
         send(msgPackageIsNeeded);
      }
   }

   /**
    * Behaviour on:
    * Storage - RampeOrderAgent
    * <p/>
    * Gets message:
    * Entrance - RampOrderAgent: REQUEST_OFFER_FROM_STORAGE_RAMP_FOR_PACKAGE
    * <p/>
    * Send message:
    * PlattformAgent - PACKAGE_SPACE_AVAILABLE
    * <p/>
    * Behaviour on a StorageRamp which sends an Request to its PlattformAgent, to ask if there is space
    * available for another package.
    *
    * @author sijakubo
    */
   private class HandleStorageRampPackageSlotEnquireBehaviour extends CyclicReceiverBehaviour {
      protected HandleStorageRampPackageSlotEnquireBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         logger.info("StorageRamp - RampOrderAgent <- ENQUIRE_STORAGE_RAMP");
         ACLMessage message = new ACLMessage(MessageType.PACKAGE_SPACE_AVAILABLE);
         //packageData
         //todo Nachricht an Packageagent anstatt an den Plattformagent
         message.setContentObject(msg.getContentObject());
         message.addUserDefinedParameter(ENQUIRING_RAMP_PARAMETER_KEY,
               msg.getUserDefinedParameter(ENQUIRING_RAMP_PARAMETER_KEY));
         message.addUserDefinedParameter("information_message_no_step", "noStep");
         AgentHelper.addReceiver(message, myAgent, RampPlattformAgent.NAME, conveyorID, szenarioID);
         send(message);
      }
   }

   /**
    * Behaviour on:
    * StorageRamp - RampOrderAgent
    * <p/>
    * Gets message:
    * PlattformAgent - RampOrderAgent: PACKAGE_SPACE_AVAILABLE
    * <p/>
    * Send message:
    * Entrance - RampOrderAgent - PACKAGE_IS_STORABLE_FROM_STORAGE_RAMP
    * <p/>
    * Behaviour which handles the response from the StorageRamps Plattform Agent
    *
    * @author sijakubo
    */
   private class HandleSpaceAvailableBehaviour extends CyclicReceiverBehaviour {
      protected HandleSpaceAvailableBehaviour(int messageType) {
         super(MessageTemplate.MatchPerformative(messageType));
      }

      @Override
      public void onMessage(ACLMessage msg) throws UnreadableException, IOException {
         String space_available = msg.getUserDefinedParameter("space_available");
         if ("1".equals(space_available)) {
            //accept offer
            ACLMessage msgReply = new ACLMessage(MessageType.PACKAGE_IS_STORABLE_FROM_STORAGE_RAMP);
            String enquiringRampConveyorId = msg.getUserDefinedParameter(ENQUIRING_RAMP_PARAMETER_KEY);
            Integer enquiringRampId = Integer.valueOf(enquiringRampConveyorId);
            logger.info("StorageRamp - RampOrderAgent -> PACKAGE_IS_STORABLE_FROM_STORAGE_RAMP, to " + enquiringRampId);
            AgentHelper.addReceiver(msgReply, myAgent, RampOrderAgent.NAME, enquiringRampId, szenarioID);
            send(msgReply);
         } else {
            //TODO decline offer, no space available
         }
      }
   }
}
