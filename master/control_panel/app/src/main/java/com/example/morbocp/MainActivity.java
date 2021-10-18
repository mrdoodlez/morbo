package com.example.morbocp;

import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    Button button_ff;
    Button button_fb;
    Button button_fl;
    Button button_fr;
    TextView console;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        button_ff = findViewById(R.id.ff_button);
        button_fb = findViewById(R.id.fb_button);
        button_fl = findViewById(R.id.fl_button);
        button_fr = findViewById(R.id.fr_button);
        console = findViewById(R.id.console);

        console.setMovementMethod(new ScrollingMovementMethod());

        button_ff.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                console.append("input event: FF\n> ");
                while(console.canScrollVertically(1)) {
                    console.scrollBy(0, 10);
                }
            }
        });

        button_fb.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                console.append("input event: FB\n> ");
                while(console.canScrollVertically(1)) {
                    console.scrollBy(0, 10);
                }
            }
        });
    }
}