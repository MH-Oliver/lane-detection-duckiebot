class Tracking
{
    //p entfernung vom ursprung und θ gleich winkel 
    //als p= x*cos(O)+y*sin(O)
/*     In der Praxis: Hesse’sche Normalform x · cos(θ) + y · sin(θ) = p
        Parameterraum (p, θ), mit 0 ≤ θ < π und −rmax ≤ r(θ) ≤ rmax
        (rmax = (1/2)*(√M²+ N²) )*/

        // bei hohen geschwindigkeiten hat sich der Kalmanfilter als schlechetr erwiesen
        //ein einfacheres verfahren letztes verwenden von (p,θ) hat sich als besser erwiesen
        //Tracking setzt das verwenden des alten (p,θ) um also Acc(p,θ)

private:
    /* data */
public:
    Tracking(/* args */);
    ~Tracking();
};


