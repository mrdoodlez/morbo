package com.example.morboremote;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Enumeration;

public class MainActivity extends AppCompatActivity {
    private enum MorboCommand {
        eCmdNone,
        eCmdLinP,
        eCmdLinM,
        eCmdAnP,
        eCmdAnM,
        eCmdVp,
        eCmdVm,
        eCmdHp,
        eCmdHm,
        eCmdStop,
        eCmdQuite
    };

    MorboCommand lastCommand = MorboCommand.eCmdNone;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        TextView statusTextView = (TextView) findViewById(R.id.statusText);
        statusTextView.setMovementMethod(new ScrollingMovementMethod());

        new Thread(new Runnable() {
            @Override
            public void run() {
                runMorboTcpClient();
            }
        }).start();
    }

    private void runMorboTcpClient() {
        String ip = "";
        try {
            Enumeration<NetworkInterface> ifaces = NetworkInterface.getNetworkInterfaces();
            while (ifaces.hasMoreElements()) {
                NetworkInterface iface = ifaces.nextElement();
                if (iface.isLoopback() || !iface.isUp())
                    continue;
                if (iface.getDisplayName().equals("wlan0")) {
                    Enumeration<InetAddress> addrs = iface.getInetAddresses();
                    while (addrs.hasMoreElements()) {
                        InetAddress addr = addrs.nextElement();
                        if (addr.getHostAddress().length() <= 15) {
                            ip = addr.getHostAddress();
                            updateStatusOnUi("local ip: " + ip + "\n");
                        }
                    }
                    break;
                }
            }


        } catch (SocketException e) {
            Log.i("T", "SocketException");
        }

        int port = 10000;

        if (!(ip.isEmpty())) {
            String[] addrs = ip.split("\\.");
            int myaddr = Integer.parseInt(addrs[3]);
            for (int a = 75; a < 0xff; a++) {
                if (a == myaddr)
                    continue;
                addrs[3] = String.valueOf(a);
                String connectAddr = addrs[0] + "." + addrs[1] + "." + addrs[2] + "." + addrs[3];
                updateStatusOnUi("Probe: " + connectAddr + " ...\n");
                try {
                    Socket socket = new Socket(connectAddr, port);
                    updateStatusOnUi("Success!\n");

                    while(true) {
                        switch (getNewCmd()) {
                            case eCmdLinP:
                                break;
                            case eCmdLinM:
                                break;
                            case eCmdAnP:
                                break;
                            case eCmdAnM:
                                break;
                            case eCmdVp:
                                break;
                            case eCmdVm:
                                break;
                            case eCmdHp:
                                break;
                            case eCmdHm:
                                break;
                            case eCmdStop:
                                break;
                            case eCmdQuite:
                                break;
                            case eCmdNone:
                            default:
                                Thread.sleep(20);
                                break;
                        }
                    }
                } catch (UnknownHostException ex) {
                    Log.i("T", "Not found");
                } catch (IOException e) {
                    Log.i("T", "I/O exception");
                } catch (InterruptedException e) {
                    Log.i("T", "Interrupt");
                }
            }
        }
    }

    private MorboCommand getNewCmd() {
        // TODO: provide lock here
        return lastCommand;
    }

    private void updateStatusOnUi(String newStatus) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                updateStatus(newStatus);
            }
        });
    }

    private void updateStatus(String newStatus) {
        TextView statusTextView = (TextView) findViewById(R.id.statusText);
        statusTextView.append("> " + newStatus);
        while (statusTextView.canScrollVertically(1))
            statusTextView.scrollBy(0, 10);
    }

    public void onClickLinearPlus(View view) {
        updateStatus("L+\n");
    }

    public void onClickLinearMinus(View view) {
        updateStatus("L-\n");
    }

    public void onClickAngularPlus(View view) {
        updateStatus("A+\n");
    }

    public void onClickAngularMinus(View view) {
        updateStatus("A-\n");
    }

    public void onClickVerticalPlus(View view) {
        updateStatus("V+\n");
    }

    public void onClickVerticalMinus(View view) {
        updateStatus("V-\n");
    }

    public void onClickHorizontalPlus(View view) {
        updateStatus("H+\n");
    }

    public void onClickHorizontalMinus(View view) {
        updateStatus("H-\n");
    }

    public void onClickStop(View view) {
        updateStatus("S\n");
    }

    public void onClickLazer(View view) {
        updateStatus("L\n");
    }
}