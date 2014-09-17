package uni.oldenburg.shared.model.statistic;

import java.io.Serializable;

/**
 * Datenmodell fuer die Paketdurchlaufzeit. Das Model enthaelt immer einen neuen Wert der gemeldet wird,
 * sobald ein Paket die Simulation verlaesst
 *
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
