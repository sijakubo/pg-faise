package uni.oldenburg.server.agent.data;

import jade.core.AID;

import java.io.Serializable;

/**
 * ContentObject to transport the PackageDate with an AID from an enquiring EntranceRamp
 *
 * @author sijakubo
 */
public class EnquiredPackageData implements Serializable {

   private AID enquirererAID;
   private PackageData packageData;

   public EnquiredPackageData(AID enquirererAID, PackageData packageData) {
      this.enquirererAID = enquirererAID;
      this.packageData = packageData;
   }

   public AID getEnquirererAID() {
      return enquirererAID;
   }

   public PackageData getPackageData() {
      return packageData;
   }
}
