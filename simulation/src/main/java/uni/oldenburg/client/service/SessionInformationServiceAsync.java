package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.AsyncCallback;

public interface SessionInformationServiceAsync {
   /**
    * @author sijakubo
    */
    void isUserLoggedInInSession(AsyncCallback<Boolean> async);
}
