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

Mit normierten $Bel(R_1)$ und $Bel(\neg R_1)$, also W.keiten mit Regenschirm an Tag 1, aus b).

$\overline{Bel}(R_2) = P(R_2 | R_1) \cdot Bel(R_1) + P(R_2 | \neg R_1) \cdot Bel(\neg R_1) = 0.7 \cdot \frac{9}{11} + 0.3 \cdot \frac{2}{11} = \frac{69}{110}$
  
$\overline{Bel}(\neg R_2) = P(\neg R_2 | R_1) \cdot Bel(R_1) + P(\neg R_2 | \neg R_1) \cdot Bel(\neg R_1) = 0.3 \cdot \frac{9}{11} + 0.7 \cdot \frac{2}{11} = \frac{41}{110}$

## d)

$Bel(R_2) = P(u_2 | R_2) \cdot \overline{Bel}(R_2) = 0.9 \cdot \frac{69}{110} = \frac{621}{1100}$

$Bel(\neg R_2) = P(u_2 | \neg R_2) \cdot \overline{Bel}(\neg R_2) = 0.2 \cdot \frac{41}{110} = \frac{41}{550}$

Normierung:

$Bel(R_2) = \frac{621 / 1100}{(621 / 1100) + (41 / 550)} = \frac{621}{703}$

$Bel(\neg R_2) = \frac{41 / 550}{(621 / 1100) + (41 / 550)} = \frac{82}{703}$

## e)

Mit normierten $Bel(R_2)$ und $Bel(\neg R_2)$ aus d).

### **Tag 3**

$\overline{Bel}(R_3) = P(R_3 | R_2) \cdot Bel(R_2) + P(R_3 | \neg R_2) \cdot Bel(\neg R_2) = 0.7 \cdot \frac{621}{703} + 0.3 \cdot \frac{82}{703} = \frac{4593}{7030}$

$\overline{Bel}(\neg R_3) = P(\neg R_3 | R_2) \cdot Bel(R_2) + P(\neg R_3 | \neg R_2) \cdot Bel(\neg R_2) = 0.3 \cdot \frac{621}{703} + 0.7 \cdot \frac{82}{703} = \frac{2437}{7030}$

Beobachteter Regenschirm:

$Bel(R_3) = P(u_3 | R_3) \cdot \overline{Bel}(R_3) = 0.9 \cdot \frac{4593}{7030} = \frac{41337}{70300} \approx 0.59$

$Bel(\neg R_3) = P(u_3 | \neg R_3) \cdot \overline{Bel}(\neg R_3) = 0.2 \cdot \frac{2437}{7030} = \frac{2437}{35150} \approx 0.07$

Normierung:

$\eta = \frac{1}{(41337 / 70300) + (2437 / 35150)} = \frac{70300}{46211} \approx 1.52$

$Bel(R_3) = \frac{41337}{70300} \cdot \eta = \frac{41337}{46211} \approx 0.89$

$Bel(\neg R_3) = \frac{2437}{35150} \cdot \eta = \frac{4874}{46211} \approx 0.11$

### **Tag 4**

$\overline{Bel}(R_4) = P(R_4 | R_3) \cdot Bel(R_3) + P(R_4 | \neg R_3) \cdot Bel(\neg R_3) = 0.7 \cdot \frac{41337}{46211} + 0.3 \cdot \frac{4874}{46211} = \frac{303981}{462110} \approx 0.66$

$\overline{Bel}(\neg R_4) = P(\neg R_4 | R_3) \cdot Bel(R_3) + P(\neg R_4 | \neg R_3) \cdot Bel(\neg R_3) = 0.3 \cdot \frac{41337}{46211} + 0.7 \cdot \frac{4874}{46211} = \frac{158129}{462110}\approx 0.34$

Beobachteter Regenschirm:

$Bel(R_4) = P(u_4 | R_4) \cdot \overline{Bel}(R_4) = 0.9 \cdot \frac{303981}{462110} = \frac{2735829}{4621100} \approx 0.59$

$Bel(\neg R_4) = P(u_4 | \neg R_4) \cdot \overline{Bel}(\neg R_4) = 0.2 \cdot \frac{158129}{462110} = \frac{158129}{2310550} \approx 0.07$

Normierung:

$\eta = \frac{1}{(41337 / 70300) + (2437 / 35150)} = \frac{4621100}{3052087} \approx 1.51$

$Bel(R_4) = \frac{2735829}{4621100} \cdot \eta = \frac{2735829}{3052087} \approx 0.90$

$Bel(\neg R_4) = \frac{158129}{2310550} \cdot \eta = \frac{316258}{3052087} \approx 0.10$

Trägt der Direktor jeden Tag einen Regenschirm bei sich. konvergiert die Wahrscheinlichkeiten für Regen gegen 0.9 und die für Nicht-Regen gegen 0.1.

## f)

Mit normierten $Bel(R_2)$ und $Bel(\neg R_2)$ aus d).

### **Tag 3**

$\overline{Bel}(R_3) = P(R_3 | R_2) \cdot Bel(R_2) + P(R_3 | \neg R_2) \cdot Bel(\neg R_2) = 0.7 \cdot \frac{621}{703} + 0.3 \cdot \frac{82}{703} = \frac{4593}{7030}$

$\overline{Bel}(\neg R_3) = P(\neg R_3 | R_2) \cdot Bel(R_2) + P(\neg R_3 | \neg R_2) \cdot Bel(\neg R_2) = 0.3 \cdot \frac{621}{703} + 0.7 \cdot \frac{82}{703} = \frac{2437}{7030}$

Da keine Beobachtung auftritt, ändert sich die W.keit nicht.

${Bel}(R_3) = \overline{Bel}(R_3) = \frac{4593}{7030}$

${Bel}(\neg R_3) = \overline{Bel}(\neg R_3) = \frac{2437}{7030}$

Normieren - keine Änderung:  

$\eta = \frac{1}{(4593 / 7030) + (2437 / 7030)} = 1$

$Bel(R_3) = \frac{4593}{7030} \cdot \eta = \frac{4593}{7030} \approx 0.65$

$Bel(\neg R_3) = \frac{2437}{7030} \cdot \eta = \frac{2437}{7030} \approx 0.35$

### **Tag 4**

$\overline{Bel}(R_4) = P(R_4 | R_3) \cdot Bel(R_3) + P(R_4 | \neg R_3) \cdot Bel(\neg R_3) = 0.7 \cdot \frac{4593}{7030} + 0.3 \cdot \frac{2437}{7030} = \frac{19731}{35150}$

$\overline{Bel}(\neg R_4) = P(\neg R_4 | R_3) \cdot Bel(R_3) + P(\neg R_4 | \neg R_3) \cdot Bel(\neg R_3) = 0.3 \cdot \frac{4593}{7030} + 0.7 \cdot \frac{2437}{7030} = \frac{15419}{35150}$

Da keine Beobachtung auftritt, ändert sich die W.keit nicht.

${Bel}(R_3) = \overline{Bel}(R_3) = \frac{19731}{35150}$

${Bel}(\neg R_3) = \overline{Bel}(\neg R_3) = \frac{15419}{35150}$

Normieren - keine Änderung:  
$\eta = \frac{1}{(19731 / 35150) + (15419 / 35150)} = 1$

$Bel(R_3) = \frac{19731}{35150} \cdot \eta = \frac{19731}{35150} \approx 0.56$

$Bel(\neg R_3) = \frac{15419}{35150} \cdot \eta = \frac{15419}{35150} \approx 0.44$

Wird der Direktor nicht mehr beobachtet, konvergieren die Wahrscheinlichkeiten für Regen und Nicht-Regen gegen 0.5.  
Ohne zusätzliche Beobachtungen nähert sich die Verteilung immer weiter dem Zufall, da der Einfluss der Beobachtungen der ersten beiden Tage stetig sinkt.
