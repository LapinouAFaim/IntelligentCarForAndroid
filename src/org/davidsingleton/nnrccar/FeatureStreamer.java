package org.davidsingleton.nnrccar;

import android.util.Log;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.net.InetSocketAddress;



/**
 * Connects to a Driver app server via TCP and streams features over the network.
 */
class FeatureStreamer {
  private Socket sock;
  private DataOutputStream dos;
  
  FeatureStreamer() {
  }
  
  void connect(String addr, int port) {
    try {
      sock = new Socket();
      sock.connect(new InetSocketAddress(addr, port), 5000);
      dos = new DataOutputStream(sock.getOutputStream());
        Log.w("tag", "SUCEESSS");

    } catch (UnknownHostException e) {
        Log.w("tag", "failllled");

        e.printStackTrace();
    } catch (IOException e) {
        Log.w("tag", "failllled IOOOOO");

        e.printStackTrace();
    }
  }
  
  void sendByteArray(byte[] send) throws IOException {
    if (dos != null) {
      dos.writeInt(send.length);
      dos.write(send, 0, send.length);
      dos.flush();
    }
  }
  
  void sendFeatures(int width, int height, byte[] send, float[] accelerometerFeatures) {
    try {
      if (dos != null) {
        dos.writeInt(width);
        dos.writeInt(height);
        dos.writeInt(accelerometerFeatures.length);
        dos.write(send, 0, send.length);
        for (int i = 0; i < accelerometerFeatures.length; i++) {
          dos.writeFloat(accelerometerFeatures[i]);
        }
        dos.flush();
      }
    } catch (IOException e) {
    }
  }

  void close() throws IOException {
    if (dos != null) {
      dos.close();
    }
    if (sock != null) {
      sock.close();
    }
  }
};
