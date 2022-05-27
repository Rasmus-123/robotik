# Aufgabe 1
a)  $\overline{Bel}(R_1) = P(R_1 | \neg{R_0}) + P(R_1 | {R_0}) = 0.3 \cdot 0.5 + 0.7 \cdot 0.5= 0.5$ \
    $\overline{Bel}(\neg R_1) = P(\neg R_1 | \neg{R_0}) + P(\neg R_1 | {R_0}) = 0.7 \cdot 0.5 + 0.3 \cdot 0.5 = 0.5$

b)  $Bel(R_1) = P(u_1 | R_1) \cdot \overline{Bel}(R_1) = 0.9 \cdot 0.5 = 0.45$ \
    $Bel(\neg R_1) = P(u_1 | \neg R_1) \cdot \overline{Bel}(\neg R_1) = 0.2 \cdot 0.5 = 0.1$ \
    \
    Bestimmung von $\eta$ für die Normierung: \
    $\eta = \frac{1}{0.45 + 0.1} = \frac{20}{11}$ \
    -> $Bel(R_1) = 0.45 \cdot \frac{20}{11} = \frac{9}{11}$ \
        $Bel(\neg R_1) = 0.1 \cdot \frac{20}{11} = \frac{2}{11}$

c)  $\overline{Bel}(R_2) = P(R_2 | R_1) \cdot Bel(R_1) + P(R_2 | \neg R_1) \cdot Bel(\neg R_1) = 0.7 \cdot \frac{9}{11} + 0.3 \cdot \frac{2}{11} = \frac{69}{110}$ \
    $\overline{Bel}(\neg R_2) = P(\neg R_2 | R_1) \cdot Bel(R_1) + P(\neg R_2 | \neg R_1) \cdot Bel(\neg R_1) = 0.3 \cdot \frac{9}{11} + 0.7 \cdot \frac{2}{11} = \frac{41}{110}$

d)  $Bel(R_2) = P(u_2 | R_2) \cdot \overline{Bel}(R_2) = 0.9 \cdot \frac{69}{110} = \frac{621}{1100}$ \
    $Bel(\neq R_2) = P(u_2 | \neq R_2) \cdot \overline{Bel}(\neq R_2) = 0.2 \cdot \frac{41}{110} = \frac{41}{550}$ \
    \
    Normierung: \
    $Bel(R_2) = \frac{621 / 1100}{(621 / 1100) + (41 / 550)} = \frac{621}{703}$ \
    $Bel(R_2) = \frac{41 / 550}{(621 / 1100) + (41 / 550)} = \frac{82}{703}$

e)  TODO

f)  TODO