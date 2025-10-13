# Lowpass Filter
The lowpass filter are used to supress the high frequency signals or noise.


The lowpass filter can be implemented using the combination of Resistance and Inductance, Resistance and capacitor or Inductor and capacitor.

We have impemented using the Resistance and Capacitor(RC).

# Lowpass Filter Design with Inverting Amplifier

**Lets say:**
\
Frequency = $\mathit{f} = 30 Hz$
\
we can use any value of capacitor value lets pick
\
$C = 470 nF$

---

### Lowpass Filter Formula

$\mathit{f} = \frac{1}{2 \pi R C}$ 
\
\
$R = \frac{1}{2 \pi C \mathit{f}}$
\
\
$R = \frac{1}{2 \pi (470n) (30)}$
\
\
$R = 11287.58 \ \Omega \approx 11k \ \Omega$

---

### Using the Inverting Amplifier

\
$Gain = A = 9$
\
\
$A = \frac{R_f}{R_{in}}$
\
\
$R_f = 11k \ \Omega$
\
\
$R_{in} = \frac{11k}{9}$
\
\
$R_{in} = 1222 \approx 1200 \ \Omega$
