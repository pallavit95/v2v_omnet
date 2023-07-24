import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;

network IntersectionScenario extends Scenario
{
    @display("bgb=1000,1000;bgl=3");
    submodules:
        rsu[13]: RSU {
            @display("p=76.049995,256.88;is=s");
        }
}
