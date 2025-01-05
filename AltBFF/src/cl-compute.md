# Formulas to compute CL loading for elevator & ailerons

We'll calculate forced for a force-feedback yoke. All forces will be based on current flight model parameters, including airspeed, trim position, etc. They are supposed to be updated on each simulation cycle.

All formulas are simplified.

$$ F_{total} = -1 * F_{elevator} + F_{trim}$$
$$ F_{elevator} = C_l * pho * V^2 * A / 2 $$
$$ C_l = (\alpha_{elevator} / alphaMax_{elevator}) * Fmax_{elevator}$$



---

Total force is computed on yoke microcontroller based on current encoder readings. We'll split total foce into a component that

$ F_{total} = -F_{spring} * pos_{elev} + F_{fixed} $



$ i = 2 $
