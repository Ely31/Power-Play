package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Random;

public class AutoConfigUtil {

    // 1 is red or left, -1 is blue or right
    int allianceSide = 1;
    public int getAllianceSide() {return allianceSide;}
    public void setAllianceSide(int allianceSide) {this.allianceSide = allianceSide;}

    public boolean sideToBool(){
        return allianceSide != 1;
    }
    public String sideToString(){
        if (allianceSide == 1) return "Left, red terminal";
        else return "Right, blue terminal";
    }

    int numCycles = 1;
    public int getNumCycles() {return numCycles;}
    public void setNumCycles(int numCycles) {this.numCycles = numCycles;}

    int parkZone = 2;
    public int getParkZone() {
        return parkZone;
    }

    public void updateParkZoneFromVisionResult(int visionResult){
        switch(visionResult){
            case 1:
                // Switch 1 and 3 if we're on blue terminal
                if (getAllianceSide() == 1){
                    parkZone = 1;
                } else {
                    parkZone = 3;
                }
                break;
            case 2:
                // We don't have to change the middle positon however
                parkZone = 2;
                break;
            case 3:
                // Switch 3 and 1 if we're on blue terminal
                if (getAllianceSide() == 1) {
                    parkZone = 3;
                } else {
                    parkZone = 1;
                }
                break;
        }
    }

    public void addTelemetry(Telemetry telemetry){
        telemetry.addLine(sideToString());
        telemetry.addData("Alliance side as integer", AutoToTele.allianceSide);
        telemetry.addData("park zone", parkZone);
        telemetry.addLine(ramdomAutoCheckMessage());
    }

    String ramdomAutoCheckMessage(){
        // Generate a random number and look up that index in the array of messages
       String[] list = {
               "CHECK THE AUTO, REMEMBER NANO FINALS 3!",
               "Run the right auto kids!",
               "Is it red? is it blue?",
               "Is it left? is it right?",
               "Are you SURE this is the program you want?",
               "Ejecute el auto correcto!",
               "올바른 자동 실행",
               "Oi mate, didjya checkit eh?",
               "What do those numbers say, hmmmm? Hmmmm?",
               "C'mon man, just take a second to read the stuff",
               "运行正确的自动",
               "Don't waste the potential of this bot",
               "How many cycles are we doin'?",
               "Where are we parkin'?",
               "Look. At. The. Side.",
               "Look at the bot, now look at the screen",
               "(insert mildly funny comment about auto)",
               "ELYYYYY...",
               "LUUUKEE...",
               ":)"
       };
       return list[new Random().nextInt(list.length)];
    }

}
