# MK2 Robot – Lettura Posizioni Iniziali Braccio Robotico

## Descrizione

Questo programma è pensato per il **robot MK2 modulo 1 con braccio robotico**.  
Il suo scopo principale è leggere le **posizioni correnti dei motori del braccio** e stamparle sul monitor seriale.  

Questo passaggio è fondamentale quando i motori vengono **disconnessi e ricollegati**, perché la posizione di riferimento `0` potrebbe cambiare. Una volta lette le posizioni, possono essere copiate nel codice principale per calibrare correttamente il braccio.

---

## Funzionamento

1. Il codice utilizza la libreria `Dynamixel_ll` per comunicare con i motori Dynamixel tramite **Serial1**.  
2. Sono definiti i motori principali:
   - `dxl`, `mot_Left_1`, `mot_Right_1` → motori principali della trazione  
   - `mot_2` … `mot_6` → motori del braccio robotico  
3. Durante `setup()`:
   - Si inizializza la comunicazione seriale con il PC e con i motori a **2 Mbps**  
   - Si disabilita la coppia sui motori (`TorqueEnable = false`) per sicurezza  
   - Si impostano i **modi di funzionamento** (Extended Position Mode)  
   - Si attiva la modalità **sync** per il controllo simultaneo di più motori  
   - Si disattiva la modalità debug  

4. Durante `loop()`:
   - Si leggono le posizioni correnti dei motori (`getPresentPosition`)  
   - Si stampano sul monitor seriale in formato leggibile, ad esempio:
  

   - Le posizioni lette possono poi essere copiate nella funzione `modc_arm_init()` del codice principale `piclowlevel.ino` per azzerare correttamente il braccio.

---

## Istruzioni per l’uso

1. Aprire il programma VS CODE.  
2. Collegare il pico del primo modulo al PC in modalità bootsel.
3. Modificare il makefile inserendo il nome corretto del pico che si sta usando. In BOARD_FQBN ?= rp2040:rp2040:rpipico o BOARD_FQBN ?= rp2040:rp2040:rpipicow a seconda dell pico.
4. Modificare il makefile inserendo il nome corretto in DESTINATION ?=  'D:\' in base a come il pc rileva il disco esterno pico.
5. Nel terminale lanciare **make compile** per compilare e scaricare il programma sul pico con **make upload bootsel**
6. Aprire il monitor seriale a **115200 baud**.  
7. Annotare o copiare i valori stampati.  
8. Inserire i valori nella funzione `MODC_ARM_INIT()` del codice in picolowlevel>picolowlevel.ino prima di eseguire qualsiasi movimento del braccio.

---

## Note

- Questo programma **non muove i motori**: serve solo per leggere le posizioni iniziali.  
- È importante eseguirlo ogni volta che i motori vengono disconnessi, per evitare movimenti imprevisti del braccio.  
- La frequenza di lettura è di 1 secondo (`delay(1000)`), ma può essere modificata se necessario.


