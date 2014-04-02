package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.rpc.RemoteService;

public interface SessionInformationServiceAsync {
   /**
    * @author sijakubo
    */
    void isUserLoggedInInSession(AsyncCallback<Boolean> async);
}
