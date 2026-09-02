package com.malviys;

import processing.core.PApplet;
import processing.opengl.PShader;

public class SimulationGPU extends PApplet {
    private PShader simpleShader;

    @Override
    public void settings() {
        size(600, 600, P2D);
    }

    @Override
    public void setup() {
        simpleShader = loadShader("gravity-lab/src/main/java/com/malviys/data/frag.glsl", "gravity-lab/src/main/java/com/malviys/data/vert.glsl");
    }

    @Override
    public void draw() {
        clear();
        shader(simpleShader);


        rect(0, 0, width, height);

        resetShader();
    }

    public static void main(String[] args) {
        PApplet.main(SimulationGPU.class, args);
    }
}
