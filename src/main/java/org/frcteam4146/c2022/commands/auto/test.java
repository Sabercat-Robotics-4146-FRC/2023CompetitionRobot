package org.frcteam4146.c2022.commands.auto;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.frcteam4146.common.control.Path;
import org.frcteam4146.common.control.SplinePathBuilder;
import org.frcteam4146.common.io.PathWriter;
import org.frcteam4146.common.math.Rotation2;
import org.frcteam4146.common.math.Vector2;

public class test {
    public static void main(String[] args) throws IOException {
        SplinePathBuilder splinePath = 
            new SplinePathBuilder(
              new Vector2(10, 5),
              new Rotation2(10.0, 10.0, true), 
              new Rotation2(1, 1, true));

        splinePath.hermite(new Vector2(5, 2), new Rotation2(50, 20, false), new Rotation2(20, 30,false));


        PathWriter w = new PathWriter(new FileWriter(new File("C:/Users/sabercatrobotics/Desktop/test.json")));

        Path path = splinePath.build();
        w.write(path);
    }
}
