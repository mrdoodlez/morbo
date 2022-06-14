package com.example.morboremote;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.view.View;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    private String status = "someText";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    private void updateStatus(String newStatus) {
        status += newStatus;
        TextView statusTextView = (TextView) findViewById(R.id.statusText);
        statusTextView.setText(status);
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