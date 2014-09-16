package uni.oldenburg.shared.model.statistic;

import java.io.Serializable;

/**
 * @author sijakubo
 */
@SuppressWarnings("serial")
public class PackageProcessingTimeDataModel implements Serializable {
   private long newProcessingTime;

   public PackageProcessingTimeDataModel() {
   }

   public PackageProcessingTimeDataModel(long newProcessingTime) {
      this.newProcessingTime = newProcessingTime;
   }

   public long getNewProcessingTime() {
      return newProcessingTime;
   }

   public void setNewProcessingTime(long newProcessingTime) {
      this.newProcessingTime = newProcessingTime;
   }
}
