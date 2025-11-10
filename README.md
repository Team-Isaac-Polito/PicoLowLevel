# MK2 Robot – Repository Controllo Braccio Robotico

Questa repository contiene il codice per il controllo del robot MK2, nel caso in cui si usino i motori Dynamixel sia per il braccio, sia per la trazione dei moduli. La struttura della repo prevede due cartelle principali:  

1. **dxl_get_position** – Progetto per leggere le posizioni correnti dei motori del braccio.  
2. **picolowlevel** – Codice principale di controllo del robot e del braccio.  

---

## ⚠️ Avvertenza importante: Azzeramento dei motori

Se i motori del braccio vengono disconnessi e ricollegati, la posizione di riferimento `0` cambia.  
Per impostare correttamente la posizione zero, segui i passaggi sottostanti prima di utilizzare il codice principale.

---
## 0. Assicurarsi di aver installato tutto il necessario presente nella guida start

## 1. Leggere le posizioni correnti

1. Aprire la cartella `dxl_get_position` in VS CODE, compilare lanciando **make compile** da terminale nella cartella dxl_get_position.  
2. Caricare il programma sul robot lanciando **make upload bootsel** da terminale nella cartella dxl_get_position, dopo aver connesso il pico del primo modulo in bootsel.  
3. Aprire il monitor seriale.  
4. I valori correnti dei motori verranno stampati sullo schermo.  

**Esempio di output seriale:**
```
getpositions0 = 2209;
getpositions0 = 1451;[3]
pos0_mot_2 = 4746;
pos0_mot_3 = 987;
pos0_mot_4 = 3121;
pos0_mot_5 = 1979;
pos0_mot_6 = 0;
```

---

## 2. Copiare i valori nel codice principale

1. Aprire la cartella `picolowlevel` e il file `piclowlevel.ino`.  
2. Cercare la funzione `MODC_ARM_INIT()`.  
3. Incollare i valori letti dal monitor seriale alla fine della funzione, sostituendo eventuali valori precedenti.  

**Esempio di inserimento:**
```
getpositions0 = 2209;
getpositions0 = 1451;[3]
pos0_mot_2 = 4746;
pos0_mot_3 = 987;
pos0_mot_4 = 3121;
pos0_mot_5 = 1979;
pos0_mot_6 = 0;
```

3. Caricare il codice principale  
   Dopo aver aggiornato la funzione `modc_arm_init()`, è possibile compilare e caricare `piclowlevel.ino` sul robot. Il braccio sarà ora correttamente azzerato e pronto per l’uso.

---

## 📝 Note

- È fondamentale eseguire prima il passaggio della lettura delle posizioni se i motori sono stati disconnessi, altrimenti il braccio non partirà dalla posizione corretta.
- Questa procedura serve solo per calibrare la posizione di zero dei motori. Una volta impostata, non è necessario rifarla a meno che i motori vengano nuovamente disconnessi.
```

