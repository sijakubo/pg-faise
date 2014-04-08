package uni.oldenburg.server.service;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import uni.oldenburg.client.service.LoggingService;

@SuppressWarnings("serial")
public class LoggingServiceImpl extends RemoteServiceServlet implements LoggingService {

   /**
    * @author sijakubo
    */
   public void logOnServer(String message, Throwable e) {
      Logger logger = Logger.getLogger(LoggingService.class);
      logger.log(Level.ERROR, message, e);
   }

   /**
    * @author sijakubo
    */
   public void logOnServer(String message) {
      Logger logger = Logger.getLogger(LoggingService.class);
      logger.log(Level.ERROR, message);
   }

   /**
    * @author sijakubo
    */
   public void logInfoOnServer(String message) {
      Logger logger = Logger.getLogger(LoggingService.class);
      logger.log(Level.INFO, message);
   }
}
