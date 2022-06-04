# Aufgabe 1

## a)

$\overline{Bel}(R_1) = P(R_1 | \neg{R_0}) \cdot P(\neg{R_0}) + P(R_1 | {R_0}) \cdot P(R_0) = 0.3 \cdot 0.5 + 0.7 \cdot 0.5= 0.5$  
$\overline{Bel}(\neg R_1) = P(\neg R_1 | \neg{R_0}) \cdot P(\neg{R_0}) + P(\neg R_1 | {R_0}) \cdot P(R_0) = 0.7 \cdot 0.5 + 0.3 \cdot 0.5 = 0.5$

## b)

$Bel(R_1) = P(u_1 | R_1) \cdot \overline{Bel}(R_1) = 0.9 \cdot 0.5 = 0.45$  
$Bel(\neg R_1) = P(u_1 | \neg R_1) \cdot \overline{Bel}(\neg R_1) = 0.2 \cdot 0.5 = 0.1$

Bestimmung von $\eta$ für die Normierung:

$\eta = \frac{1}{0.45 + 0.1} = \frac{20}{11}$

-> $Bel(R_1) = 0.45 \cdot \frac{20}{11} = \frac{9}{11}$

$Bel(\neg R_1) = 0.1 \cdot \frac{20}{11} = \frac{2}{11}$

## c)

$\overline{Bel}(R_2) = P(R_2 | R_1) \cdot Bel(R_1) + P(R_2 | \neg R_1) \cdot Bel(\neg R_1) = 0.7 \cdot \frac{9}{11} + 0.3 \cdot \frac{2}{11} = \frac{69}{110}$

$\overline{Bel}(\neg R_2) = P(\neg R_2 | R_1) \cdot Bel(R_1) + P(\neg R_2 | \neg R_1) \cdot Bel(\neg R_1) = 0.3 \cdot \frac{9}{11} + 0.7 \cdot \frac{2}{11} = \frac{41}{110}$

---

Mit ${Bel}(R_1)$ und $Bel(\neg R_1)$ aus b)

$\overline{Bel}(R_2) = P(R_2 | R_1) \cdot Bel(R_1) + P(R_2 | \neg R_1) \cdot Bel(\neg R_1) = 0.7 \cdot 0.45 + 0.3 \cdot 0.1 = 0.345$

$\overline{Bel}(\neg R_2) = P(\neg R_2 | R_1) \cdot Bel(R_1) + P(\neg R_2 | \neg R_1) \cdot Bel(\neg R_1) = 0.3 \cdot 0.45 + 0.7 \cdot 0.1 = 0.205$

## d)

$Bel(R_2) = P(u_2 | R_2) \cdot \overline{Bel}(R_2) = 0.9 \cdot \frac{69}{110} = \frac{621}{1100}$

$Bel(\neg R_2) = P(u_2 | \neg R_2) \cdot \overline{Bel}(\neg R_2) = 0.2 \cdot \frac{41}{110} = \frac{41}{550}$

Normierung:

$Bel(R_2) = \frac{621 / 1100}{(621 / 1100) + (41 / 550)} = \frac{621}{703}$

$Bel(R_2) = \frac{41 / 550}{(621 / 1100) + (41 / 550)} = \frac{82}{703}$

---

Mit $\overline{Bel}(R_2)$ und $\overline{Bel}(\neg R_2)$ aus c)

$Bel(R_2) = P(u_2 | R_2) \cdot \overline{Bel}(R_2) = 0.9 \cdot 0.345 = 0.3105$

$Bel(\neg R_2) = P(u_2 | \neg R_2) \cdot \overline{Bel}(\neg R_2) = 0.2 \cdot 0.205 = 0.041$

## e)  TODO


## f)

$\overline{Bel}(R_3) = P(R_3 | R_2) \cdot Bel(R_2) + P(R_3 | \neg R_2) \cdot Bel(\neg R_2) = 0.7 \cdot 0.3105 + 0.3 \cdot 0.041 = 0.22965$

$\overline{Bel}(\neg R_3) = P(\neg R_3 | R_2) \cdot Bel(R_2) + P(\neg R_3 | \neg R_2) \cdot Bel(\neg R_2) = 0.3 \cdot 0.3105 + 0.7 \cdot 0.041 = 0.12185$

${Bel}(R_3) = \overline{Bel}(R_3) = 0.22965$

${Bel}(\neg R_3) = \overline{Bel}(\neg R_3) = 0.12185$

$\overline{Bel}(R_4) = P(R_4 | R_3) \cdot Bel(R_3) + P(R_4 | \neg R_3) \cdot Bel(\neg R_3) = 0.7 \cdot 0.22965 + 0.3 \cdot 0.12185 = 0.19731$

$\overline{Bel}(\neg R_4) = P(\neg R_4 | R_3) \cdot Bel(R_3) + P(\neg R_4 | \neg R_3) \cdot Bel(\neg R_2) = 0.3 \cdot 0.22965 + 0.7 \cdot 0.12185 = 0.15419$

${Bel}(R_4) = \overline{Bel}(R_4) = 0.19731$

${Bel}(\neg R_4) = \overline{Bel}(\neg R_4) = 0.15419$

Beide Bels(R und nichtR) konvergieren gegen etwa 0.1756 (oder so).
