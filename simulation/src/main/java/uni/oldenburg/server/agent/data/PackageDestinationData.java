package uni.oldenburg.server.agent.data;

/**
 * ContentObject to transport a rampDestination and a packageId
 *
 * @author sijakubo
 */
public class PackageDestinationData {
   private int rampDestination;
   private int packageId;

   public PackageDestinationData(int rampDestination, int packageId) {
      this.rampDestination = rampDestination;
      this.packageId = packageId;
   }

   public int getRampDestination() {
      return rampDestination;
   }

   public int getPackageId() {
      return packageId;
   }
}
