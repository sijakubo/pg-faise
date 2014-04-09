package uni.oldenburg.client.util;

import com.google.gwt.user.client.rpc.AsyncCallback;

/**
 * @author sijakubo
 */
public class EmptyAsyncCallback implements AsyncCallback<Void> {
   public void onFailure(Throwable caught) {
      //doNothing
   }

   public void onSuccess(Void result) {
      //doNothing
   }
}
