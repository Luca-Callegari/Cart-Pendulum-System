# Controllo e Regolazione di un Pendolo Inverso su Carrello

Questo progetto si occupa della modellizzazione, linearizzazione e progettazione di sistemi di controllo per un pendolo inverso montato su un carrello mobile soggetto a segnali esogeni (disturbi e riferimenti). Sono state progettate e analizzate due differenti architetture di controllo: **Full Information** (informazione completa) e **Measurement Information** (controllo basato solo su misure).

---

## 1. Formulazione del Problema ed Equazioni della Dinamica
Il sistema è costituito da un carrello che si muove lungo una guida orizzontale e da un pendolo vincolato al carrello stesso. Il comportamento dinamico del sistema non lineare è governato dalle seguenti equazioni differenziali:

$$M\dot{s}+F\dot{s}-\mu=d_{1}$$
$$\dot{\phi}-\frac{g}{L}\sin\phi+\frac{1}{L}\dot{s}\cos\phi=0$$

### Parametri del Sistema
* **Massa del carrello ($M$):** $1 \text{ kg}$
* **Lunghezza del pendolo ($L$):** $1 \text{ m}$
* **Coefficiente di attrito ($F$):** $1 \text{ kg/s}$
* **Accelerazione di gravità ($g$):** $9.81 \text{ m/s}^2$

### Variabili in Gioco
* $s(t)$: Posizione lineare del carrello lungo la guida.
* $\phi(t)$: Angolo di inclinazione del pendolo rispetto alla verticale superiore (con $\phi = 0$ che rappresenta la posizione verticale instabile).
* $\mu(t)$: Forza esterna applicata al carrello, che funge da ingresso di controllo $u(t)$.
* $d_{1}(t)$: Disturbo di forza esterno e costante agente sul carrello.

---

## 2. Punti di Equilibrio e Linearizzazione
L'analisi del sistema si concentra sull'intorno dell'equilibrio instabile, ovvero quando il pendolo si trova perfettamente verticale verso l'alto ed il carrello è fermo. 

Definendo il vettore di stato del sistema tramite la posizione del carrello, la sua velocità, l'angolo del pendolo e la sua velocità angolare, le equazioni differenziali sono state linearizzate attorno al punto di lavoro instabile originario. 

Attraverso il calcolo della matrice di controllabilità del sistema lineare così ottenuto, è stato verificato che il sistema gode della proprietà di **controllabilità completa** (la matrice associata risulta a rango pieno). Ciò garantisce matematicamente la possibilità di modificare il comportamento del sistema e stabilizzarlo tramite un'opportuna legge di controllo.

---

## 3. Il Problema di Regolazione e l'Esosistema
L'obiettivo principale del progetto è la regolazione dell'errore di tracking, definito come la differenza tra la posizione del carrello e un segnale di riferimento esterno sinusoidale:

$$e(t) = s(t) - d_2(t)$$

Il controllore deve essere in grado di soddisfare simultaneamente due requisiti a regime asintotico:
1. **Inseguimento del riferimento:** Il carrello deve muoversi seguendo una traiettoria sinusoidale data da $d_2(t) = \alpha \sin(\omega t)$.
2. **Reiezione del disturbo:** Il sistema deve neutralizzare l'effetto del disturbo costante $d_1$ agente sul carrello.

I segnali esogeni (disturbo e riferimento) sono stati modellizzati matematicamente definendo un **esosistema autonomo**, la cui dinamica racchiude sia il modo costante del disturbo sia i modi complessi coniugati della sinusoide alla frequenza $\omega$.

---

## 4. Architetture di Controllo Progettate

### 🛡️ Full Information (Informazione Completa)
In questa configurazione si ipotizza che sia l'intero stato del sistema sia le variabili dell'esosistema siano direttamente accessibili e misurabili. La legge di controllo viene strutturata come combinazione lineare dello stato del pendolo e dei segnali esogeni.

La risolubilità di questo schema di controllo è stata verificata analiticamente attraverso la risoluzione delle **equazioni di Francis-Byrnes-Isidori (FBI)**. La consistenza matematica e la stabilità del sistema a ciclo chiuso sono state validate applicando il **Lemma di Hautus** per il test di rango sui modi dell'esosistema, confermando che il sistema è pienamente regolabile sotto queste ipotesi.

### 🔍 Measurement Information (Controllo basato su Misure)
Nel caso più realistico in cui non sia possibile misurare direttamente tutte le variabili di stato, il controllo deve basarsi esclusivamente sulle uscite misurate. L'analisi iniziale ha evidenziato che il semplice utilizzo dell'errore di tracking classico non permette di ottenere l'osservabilità del sistema complessivo (il sistema esteso perde di rango).

Per superare questo limite strutturale, è stato introdotto un **errore di misura modificato *ad hoc*** che include, oltre all'errore di posizione, anche la misura diretta dell'angolo del pendolo. Grazie a questo accorgimento, la matrice di osservabilità del sistema esteso soddisfa la condizione di rango pieno. È stato quindi possibile progettare un **controllore dinamico basato su stimatore (osservatore dello stato e del disturbo)** in grado di ricostruire le informazioni mancanti e garantire la regolazione del sistema.

---
**Autore:** Di Luca Callegari   
