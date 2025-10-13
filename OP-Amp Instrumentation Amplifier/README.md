![Instrumental Amplifier](./Images/Instrumental%20Amplifier.jpg)

***Gain of an instrumental amplifier***

<p align = "center">
$Gain = \left(1+\frac{R_{2}}{R_{gain}}\right)\left(\frac{R_{4}}{R_{3}}\right)$
</p>

lets say we need a gain of $110V/V$
\
The calculation can be simplified by keeping the gain of both stages equal.
This can be achieved by setting the feedback resistors of both stages to the same value.
Any appropriate value can be selected;
\
we have chosen such a value.
\
$R_{2} = R_{4} = 5k\Omega$
\
if we use $R_{g} = 1k\Omega$
<p align = "center">

$$R_3 = \left( 1 + \frac{2R_2}{R_g} \right) \left( \frac{R_4}{G} \right)$$
$$R_3 = \left( 1 + 2 * \frac{5000}{1000} \right) \left( \frac{5000}{110} \right)$$
$$R_3 = (1 + 2 * 5) \left( \frac{500}{11} \right)$$
$$R_3 = (11) \left( \frac{500}{11} \right)$$
$$R_3 = 500~\Omega$$
</p>


