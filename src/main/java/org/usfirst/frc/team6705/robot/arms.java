public class Arms{
 static DoubleSolenoid doubleSolenoid = new DoubleSolenoid(5,9);
public static void open(){

doubleSolenoid.set(DoubleSolenoid.Value.kForward);



}

public static void close(){
doubleSolenoid.set(DoubleSolenoid.Value.kReverse);

}    
    
    }