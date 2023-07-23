import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;

network IntersectionScenario extends Scenario
{
    @display("bgb=663,399;bgl=2");
    submodules:
        rsu[4]: RSU {
            @display("p=313.3,296.4");
        }
}
