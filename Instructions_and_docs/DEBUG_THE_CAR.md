# Debugging: RC-bil starter ikke

Dette dokument beskriver de mest almindelige årsager til, at RC-bilen ikke starter eller ikke fungerer korrekt. Indholdet er baseret på praktiske debugging-erfaringer samt kendte tekniske begrænsninger i systemet. Problemerne er prioriteret efter hvor ofte de typisk opstår.

---

# Prioriteret fejlsøgningsliste

1. [Strømforsyning og batteri](#strømforsyning-og-batteri)  
2. [Spændingsfald under belastning](#spændingsfald-under-belastning)  
3. [Flight controller under boot](#flight-controller-under-boot)  
4. [QOpenHD UI viser forkert status](#qopenhd-ui-viser-forkert-status)  
5. [Forkert air unit eller frekvenskonflikt](#forkert-air-unit-eller-frekvenskonflikt)  
6. [Løse forbindelser og elektrisk støj](#løse-forbindelser-og-elektrisk-støj)  
7. [Servo og styringsproblemer](#servo-og-styringsproblemer)  
8. [Manglende dokumentation for servo](#manglende-dokumentation-for-servo)  
9. [Software- og kodeproblemer](#software-og-kodeproblemer)  
10. [Defekte komponenter](#defekte-komponenter)  

---

# Strømforsyning og batteri

Når RC-bilen anvendes, er det vigtigt at sikre, at batteriet kan levere tilstrækkelig strøm til både air unit og flight controller. Dette kan kontrolleres i QOpenHD, når air unit er tilsluttet.

Batterierne har en relativt kort levetid, og selvom systemet kan oprette forbindelse til air unit, kan for lav spænding medføre, at flight controller ikke fungerer korrekt.

En praktisk løsning er at forsyne air unit direkte med en ekstern strømforsyning i stedet for batteriet. Batteriet kan derefter bruges til at forsyne flight controller, kamera og accelerometer. På den måde reduceres belastningen på batteriet, hvilket kan øge systemets stabilitet.

---

# Spændingsfald under belastning

Under debugging blev det observeret, at batteriet kunne se ud til at fungere korrekt, når spændingen blev målt uden belastning.

Når systemet derimod blev sat under belastning, faldt spændingen hurtigt til et niveau, hvor flight controller ikke længere fungerede korrekt. Der blev ikke draget en fuldstændig konklusion, da måleudstyret var begrænset, men observationerne tyder på et klassisk spændingsfald under belastning.

Dette betyder, at et batteri godt kan se funktionelt ud under måling, men stadig være utilstrækkeligt i praksis.

---

# Flight controller under boot

Flight controlleren skal være aktiv, når air unit starter. Hvis air unit booter før flight controlleren, kan den ikke finde flight controlleren.

Dette kan føre til, at systemet ikke registrerer flight controlleren, og dermed fungerer styring og telemetri ikke korrekt.

Hvis flight controller ikke registreres, vil systemstatus vise følgende:

![](https://github.com/SWIGX/fanatec-FC-setup/blob/main/Images/Not_Connected.png?raw=true)

Hvis forbindelsen er korrekt, vil status vise:

`FOUND:1`

![](https://github.com/SWIGX/fanatec-FC-setup/blob/main/Images/All_Connected.png?raw=true)

Selvom flight controller-status til højre i interfacet bliver grå, kan controlleren stadig være aktiv og fungere korrekt.

---

# QOpenHD UI viser forkert status

Brugerinterfacet i QOpenHD viser ikke altid korrekt systemstatus.

I nogle tilfælde blev flight controller vist som frakoblet i interfacet, selvom den faktisk var tilsluttet og fungerede.

Den mest pålidelige indikator for forbindelsen er derfor feltet under systemstatus, hvor der står:

`FOUND:1`

Hvis denne værdi vises, er flight controller normalt korrekt fundet af systemet.

---

# Forkert air unit eller frekvenskonflikt

Systemet skal kommunikere med den korrekte air unit. Hvis flere grupper anvender air units på samme frekvens, kan ground station i nogle tilfælde forbinde til den forkerte enhed.

Dette kan være vanskeligt at opdage, især hvis de andre systemer bruger samme Raspberry Pi-model.

For at reducere risikoen bør man vælge en frekvens med så lav aktivitet som muligt.

---

# Løse forbindelser og elektrisk støj

Opsætningen indeholder mange kabler, som ofte ligger tæt sammen. Dette kan føre til forskellige hardwareproblemer, herunder løse forbindelser, elektrisk støj eller fejl i kablerne.

Derfor er det vigtigt at have et klart overblik over alle forbindelser og kontrollere, at kablerne sidder korrekt og er elektrisk funktionelle.

Et uorganiseret kabelsetup kan skabe fejl, som er svære at identificere og reproducere.

---

# Servo og styringsproblemer

Et af de mest markante problemer under udviklingen var ujævn styring. Bilen drejede mere til højre end til venstre.

For at analysere problemet blev en Analog Discovery 2 brugt til at måle PWM-signalerne.

Først blev systemet testet med den normale fjernbetjening, hvor problemet ikke opstod. Derefter blev PWM-signalet fra rat-controlleren sammenlignet med signalet fra fjernbetjeningen.

Der blev analyseret mange forskellige PWM-signaler med forskellige steering trims. Under testen blev der observeret uventet opførsel, og det var ikke muligt at finde et tydeligt mønster i signalerne.

---

# Manglende dokumentation for servo

Dokumentationen for RC-bilen indeholder ingen detaljer om den anvendte servo.

Derfor var det nødvendigt at finde den korrekte mapping mellem raw channel data og PWM-signalet, der sendes til motoren. Dette blev gjort gennem en række forsøg med forskellige parameterkombinationer.

---

# Servo løsning

Problemet blev løst ved at ændre initialiseringen af servoen i koden.

Følgende konfiguration blev anvendt:

Denne opsætning ændrer PWM-intervallet og forbedrede styringen markant.

Derudover blev der implementeret en funktion til at justere steering trim med plus- og minusknapper, så systemet kan kalibreres efter behov.

Efter denne ændring fungerede styringen korrekt med rattet.

---

# Software og kodeproblemer

Hvis hardwareproblemer kan udelukkes, kan fejlen skyldes software. Eksempler kan være forkert PWM-mapping, forkert initialisering af servo eller timingproblemer under boot.

Softwarefejl bør dog først undersøges efter strømforsyning, forbindelser og hardware er verificeret.

---

# Defekte komponenter

Hvis ingen af de tidligere fejlårsager forklarer problemet, kan det skyldes defekt hardware.

Mulige fejl inkluderer en defekt flight controller, et beskadiget batteri, en defekt air unit eller beskadigede kabler.

Statistisk set er denne type fejl dog mindre almindelig end problemer med strømforsyning eller forbindelser.


