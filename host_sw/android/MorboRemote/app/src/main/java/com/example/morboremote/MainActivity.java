package com.example.morboremote;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
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
        eCmdLz,
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
            boolean roverDetected = false;
            for (int a = 75; a < 0xff && !roverDetected; a++) {
                if (a == myaddr)
                    continue;
                addrs[3] = String.valueOf(a);
                String connectAddr = addrs[0] + "." + addrs[1] + "." + addrs[2] + "." + addrs[3];
                try {
                    updateStatusOnUi("Probe: " + connectAddr + " ...\n");
                    Socket socket = new Socket(connectAddr, port);
                    roverDetected = true;
                    updateStatusOnUi("Rover detected\n");
                    OutputStream output = socket.getOutputStream();

                    while(true) {
                        byte[] data = "mbx".getBytes(StandardCharsets.US_ASCII);
                        MorboCommand cmd = getNewCmd();
                        switch (cmd) {
                            case eCmdLinP:
                                data[2] = 119;
                                break;
                            case eCmdLinM:
                                data[2] = 120;
                                break;
                            case eCmdAnP:
                                data[2] = 97;
                                break;
                            case eCmdAnM:
                                data[2] = 100;
                                break;
                            case eCmdVp:
                                data[2] = 117;
                                break;
                            case eCmdVm:
                                data[2] = 110;
                                break;
                            case eCmdHp:
                                data[2] = 104;
                                break;
                            case eCmdHm:
                                data[2] = 107;
                                break;
                            case eCmdStop:
                                data[2] = 115;
                                break;
                            case eCmdLz:
                                data[2] = 106;
                                break;
                            default:
                                break;
                        }
                        if (cmd != MorboCommand.eCmdNone) {
                            setNewCmd(MorboCommand.eCmdNone);
                            output.write(data, 0, 3);
                            if (cmd == MorboCommand.eCmdQuite)
                                break;
                        } else {
                            Thread.sleep(20);
                        }
                    }
                    updateStatusOnUi("Quite\n");
                    socket.close();
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

    private void setNewCmd(MorboCommand cmd) {
        // TODO: provide lock here
        lastCommand = cmd;
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
        setNewCmd(MorboCommand.eCmdLinP);
    }

    public void onClickLinearMinus(View view) {
        setNewCmd(MorboCommand.eCmdLinM);
    }

    public void onClickAngularPlus(View view) {
        setNewCmd(MorboCommand.eCmdAnP);
    }

    public void onClickAngularMinus(View view) {
        setNewCmd(MorboCommand.eCmdAnM);
    }

    public void onClickVerticalPlus(View view) {
        setNewCmd(MorboCommand.eCmdVp);
    }

    public void onClickVerticalMinus(View view) {
        setNewCmd(MorboCommand.eCmdVm);
    }

    public void onClickHorizontalPlus(View view) {
        setNewCmd(MorboCommand.eCmdHp);
    }

    public void onClickHorizontalMinus(View view) {
        setNewCmd(MorboCommand.eCmdHm);
    }

    public void onClickStop(View view) {
        setNewCmd(MorboCommand.eCmdStop);
    }

    public void onClickLazer(View view) {
        setNewCmd(MorboCommand.eCmdLz);
    }

    public void onClickQuite(View view) {
        setNewCmd(MorboCommand.eCmdQuite);
    }
}